"""Known-object tracking and immutable Phase-4 world snapshots."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any

import numpy as np


class DynamicWorldError(ValueError):
    """An observation cannot be represented by the Phase-4 world model."""


def _vector(value: Any, size: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise DynamicWorldError(f"{name} must be {size} finite numbers")
    return array


def _quaternion_xyzw(value: Any) -> tuple[float, float, float, float]:
    quaternion = _vector(value, 4, "orientation_xyzw")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise DynamicWorldError("orientation_xyzw has zero norm")
    return tuple((quaternion / norm).tolist())


def _validate_local_sdf(value: Any) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise DynamicWorldError("local_sdf must be an object")
    shape_type = value.get("type")
    if shape_type == "sphere":
        radius = float(value.get("radius", 0.0))
        if not math.isfinite(radius) or radius <= 0.0:
            raise DynamicWorldError("sphere radius must be positive")
        return {"type": "sphere", "radius": radius}
    if shape_type == "box":
        size = _vector(value.get("size_xyz"), 3, "box size_xyz")
        if np.any(size <= 0.0):
            raise DynamicWorldError("box size_xyz must be positive")
        return {"type": "box", "size_xyz": size.tolist()}
    if shape_type == "capsule":
        radius = float(value.get("radius", 0.0))
        length = float(value.get("length", 0.0))
        if not all(math.isfinite(item) and item > 0.0 for item in (radius, length)):
            raise DynamicWorldError("capsule radius and length must be positive")
        return {"type": "capsule", "radius": radius, "length": length}
    raise DynamicWorldError("local_sdf.type must be sphere, box, or capsule")


@dataclass(frozen=True)
class DynamicObjectSnapshot:
    object_id: str
    local_sdf: dict[str, Any]
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity: tuple[float, float, float]
    covariance_6x6: tuple[float, ...]
    inflation_mode: str
    base_inflation_m: float
    horizon_inflation_rate_m_s: float

    def to_worker_dict(self) -> dict[str, Any]:
        return {
            "id": self.object_id,
            "local_sdf": self.local_sdf,
            "pose": {
                "position": list(self.position),
                "orientation_xyzw": list(self.orientation_xyzw),
            },
            "linear_velocity": list(self.linear_velocity),
            "covariance_6x6": list(self.covariance_6x6),
            "inflation": {
                "mode": self.inflation_mode,
                "base_m": self.base_inflation_m,
                "horizon_rate_m_s": self.horizon_inflation_rate_m_s,
            },
        }


@dataclass(frozen=True)
class DynamicWorldSnapshot:
    version: int
    frame_id: str
    stamp_unix_ns: int
    valid_until_unix_ns: int
    objects: tuple[DynamicObjectSnapshot, ...]

    def to_worker_dict(self) -> dict[str, Any]:
        return {
            "world_version": self.version,
            "frame_id": self.frame_id,
            "stamp_unix_ns": self.stamp_unix_ns,
            "valid_until_unix_ns": self.valid_until_unix_ns,
            "objects": [item.to_worker_dict() for item in self.objects],
        }


class ConstantVelocityKalmanFilter:
    """Linear [position, velocity] Kalman filter with white acceleration noise."""

    def __init__(
        self,
        position: np.ndarray,
        stamp_unix_ns: int,
        position_covariance: np.ndarray,
        *,
        initial_velocity_std_m_s: float,
        process_acceleration_std_m_s2: float,
    ) -> None:
        self.state = np.concatenate((position, np.zeros(3, dtype=np.float64)))
        self.covariance = np.zeros((6, 6), dtype=np.float64)
        self.covariance[:3, :3] = position_covariance
        self.covariance[3:, 3:] = np.eye(3) * initial_velocity_std_m_s**2
        self.stamp_unix_ns = int(stamp_unix_ns)
        self.process_variance = float(process_acceleration_std_m_s2) ** 2

    def _predict(self, stamp_unix_ns: int) -> None:
        if stamp_unix_ns < self.stamp_unix_ns:
            raise DynamicWorldError("object observations must not move backwards in time")
        dt = (stamp_unix_ns - self.stamp_unix_ns) * 1e-9
        if dt == 0.0:
            return
        identity = np.eye(3)
        transition = np.block([[identity, dt * identity], [np.zeros((3, 3)), identity]])
        process = self.process_variance * np.block(
            [
                [0.25 * dt**4 * identity, 0.5 * dt**3 * identity],
                [0.5 * dt**3 * identity, dt**2 * identity],
            ]
        )
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process
        self.stamp_unix_ns = int(stamp_unix_ns)

    def update(
        self,
        position: np.ndarray,
        stamp_unix_ns: int,
        measurement_covariance: np.ndarray,
    ) -> None:
        self._predict(stamp_unix_ns)
        observation = np.hstack((np.eye(3), np.zeros((3, 3))))
        innovation = position - observation @ self.state
        innovation_covariance = observation @ self.covariance @ observation.T + measurement_covariance
        gain = np.linalg.solve(
            innovation_covariance.T,
            (self.covariance @ observation.T).T,
        ).T
        self.state = self.state + gain @ innovation
        # Joseph form preserves symmetry/positive semidefiniteness.
        residual = np.eye(6) - gain @ observation
        self.covariance = (
            residual @ self.covariance @ residual.T
            + gain @ measurement_covariance @ gain.T
        )

    def predict_copy(self, stamp_unix_ns: int) -> tuple[np.ndarray, np.ndarray]:
        state = self.state.copy()
        covariance = self.covariance.copy()
        previous_stamp = self.stamp_unix_ns
        self._predict(stamp_unix_ns)
        predicted_state = self.state.copy()
        predicted_covariance = self.covariance.copy()
        self.state, self.covariance, self.stamp_unix_ns = state, covariance, previous_stamp
        return predicted_state, predicted_covariance


@dataclass
class _Track:
    filter: ConstantVelocityKalmanFilter
    local_sdf: dict[str, Any]
    orientation_xyzw: tuple[float, float, float, float]
    inflation_mode: str
    base_inflation_m: float
    horizon_inflation_rate_m_s: float


class DynamicWorldManager:
    """Turn position observations into versioned CV predictions."""

    def __init__(
        self,
        *,
        frame_id: str = "fr3_link0",
        prediction_horizon_s: float = 12.0,
        max_objects: int = 16,
        initial_velocity_std_m_s: float = 0.25,
        process_acceleration_std_m_s2: float = 0.01,
        default_position_std_m: float = 0.01,
        default_inflation_mode: str = "covariance",
        default_base_inflation_m: float = 0.01,
        default_horizon_inflation_rate_m_s: float = 0.01,
        initial_version: int | None = None,
    ) -> None:
        if prediction_horizon_s <= 0.0 or max_objects < 1:
            raise ValueError("prediction horizon and max_objects must be positive")
        self.frame_id = frame_id
        self.prediction_horizon_s = float(prediction_horizon_s)
        self.max_objects = int(max_objects)
        self.initial_velocity_std_m_s = float(initial_velocity_std_m_s)
        self.process_acceleration_std_m_s2 = float(process_acceleration_std_m_s2)
        self.default_position_std_m = float(default_position_std_m)
        self.default_inflation_mode = default_inflation_mode
        self.default_base_inflation_m = float(default_base_inflation_m)
        self.default_horizon_inflation_rate_m_s = float(default_horizon_inflation_rate_m_s)
        self._tracks: dict[str, _Track] = {}
        self._version = time.time_ns() if initial_version is None else int(initial_version)
        self._snapshot: DynamicWorldSnapshot | None = None
        self._lock = threading.Lock()

    @property
    def snapshot(self) -> DynamicWorldSnapshot | None:
        with self._lock:
            return self._snapshot

    def update(self, observation: dict[str, Any]) -> DynamicWorldSnapshot:
        if not isinstance(observation, dict):
            raise DynamicWorldError("observation must be an object")
        frame_id = observation.get("frame_id")
        if frame_id != self.frame_id:
            raise DynamicWorldError(f"frame_id must be {self.frame_id!r}")
        stamp = observation.get("stamp_unix_ns")
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp < 0:
            raise DynamicWorldError("stamp_unix_ns must be a non-negative integer")
        objects = observation.get("objects")
        if not isinstance(objects, list) or len(objects) > self.max_objects:
            raise DynamicWorldError(f"objects must contain at most {self.max_objects} entries")

        seen: set[str] = set()
        with self._lock:
            for item in objects:
                if not isinstance(item, dict):
                    raise DynamicWorldError("each observed object must be an object")
                object_id = item.get("id")
                if not isinstance(object_id, str) or not object_id or object_id in seen:
                    raise DynamicWorldError("object ids must be unique non-empty strings")
                seen.add(object_id)
                position = _vector(item.get("position"), 3, "position")
                covariance_value = item.get("position_covariance_3x3")
                if covariance_value is None:
                    measurement_covariance = np.eye(3) * self.default_position_std_m**2
                else:
                    measurement_covariance = np.asarray(covariance_value, dtype=np.float64).reshape(3, 3)
                    if not np.isfinite(measurement_covariance).all():
                        raise DynamicWorldError("position covariance contains NaN or Inf")
                track = self._tracks.get(object_id)
                if track is None:
                    inflation_mode = str(
                        item.get("inflation_mode", self.default_inflation_mode)
                    )
                    base_inflation = float(
                        item.get("base_inflation_m", self.default_base_inflation_m)
                    )
                    horizon_rate = float(
                        item.get(
                            "horizon_inflation_rate_m_s",
                            self.default_horizon_inflation_rate_m_s,
                        )
                    )
                    if inflation_mode not in ("linear", "covariance"):
                        raise DynamicWorldError(
                            "inflation_mode must be linear or covariance"
                        )
                    if not all(
                        math.isfinite(value) and value >= 0.0
                        for value in (base_inflation, horizon_rate)
                    ):
                        raise DynamicWorldError(
                            "inflation values must be finite and non-negative"
                        )
                    track = _Track(
                        filter=ConstantVelocityKalmanFilter(
                            position,
                            stamp,
                            measurement_covariance,
                            initial_velocity_std_m_s=self.initial_velocity_std_m_s,
                            process_acceleration_std_m_s2=self.process_acceleration_std_m_s2,
                        ),
                        local_sdf=_validate_local_sdf(item.get("local_sdf")),
                        orientation_xyzw=_quaternion_xyzw(
                            item.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
                        ),
                        inflation_mode=inflation_mode,
                        base_inflation_m=base_inflation,
                        horizon_inflation_rate_m_s=horizon_rate,
                    )
                    self._tracks[object_id] = track
                else:
                    if _validate_local_sdf(item.get("local_sdf", track.local_sdf)) != track.local_sdf:
                        raise DynamicWorldError("an object's known local_sdf cannot change at runtime")
                    track.filter.update(position, stamp, measurement_covariance)
                    track.orientation_xyzw = _quaternion_xyzw(
                        item.get("orientation_xyzw", track.orientation_xyzw)
                    )

            # Objects absent from this complete observation are inactive.
            self._tracks = {key: value for key, value in self._tracks.items() if key in seen}
            snapshots = []
            for object_id in sorted(self._tracks):
                track = self._tracks[object_id]
                state, covariance = track.filter.predict_copy(stamp)
                snapshots.append(
                    DynamicObjectSnapshot(
                        object_id=object_id,
                        local_sdf=track.local_sdf,
                        position=tuple(state[:3].tolist()),
                        orientation_xyzw=track.orientation_xyzw,
                        linear_velocity=tuple(state[3:].tolist()),
                        covariance_6x6=tuple(covariance.reshape(-1).tolist()),
                        inflation_mode=track.inflation_mode,
                        base_inflation_m=track.base_inflation_m,
                        horizon_inflation_rate_m_s=track.horizon_inflation_rate_m_s,
                    )
                )
            self._version += 1
            self._snapshot = DynamicWorldSnapshot(
                version=self._version,
                frame_id=self.frame_id,
                stamp_unix_ns=stamp,
                valid_until_unix_ns=stamp + int(self.prediction_horizon_s * 1e9),
                objects=tuple(snapshots),
            )
            return self._snapshot
