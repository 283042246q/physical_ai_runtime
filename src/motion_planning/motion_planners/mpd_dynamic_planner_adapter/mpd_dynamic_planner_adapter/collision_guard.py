"""Fast dynamic-only validation of MPD-exported collision-sphere trajectories."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .dynamic_world import DynamicObjectSnapshot, DynamicWorldSnapshot


def _rotation_xyzw(value) -> np.ndarray:
    x, y, z, w = value
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _inflation(item: DynamicObjectSnapshot, dt: np.ndarray, covariance_sigma: float, process_variance: float):
    if item.inflation_mode == "linear":
        return item.base_inflation_m + item.horizon_inflation_rate_m_s * dt
    covariance = np.asarray(item.covariance_6x6, dtype=np.float64).reshape(6, 6)
    p_pp, p_pv = covariance[:3, :3], covariance[:3, 3:]
    p_vp, p_vv = covariance[3:, :3], covariance[3:, 3:]
    output = np.empty_like(dt)
    for index, horizon in enumerate(dt):
        p_position = p_pp + horizon * (p_pv + p_vp) + horizon**2 * p_vv
        p_position = p_position + np.eye(3) * process_variance * horizon**3 / 3.0
        output[index] = item.base_inflation_m + covariance_sigma * math.sqrt(
            max(0.0, float(np.linalg.eigvalsh(p_position).max()))
        )
    return output


def _local_sdf(points: np.ndarray, item: DynamicObjectSnapshot) -> np.ndarray:
    shape = item.local_sdf
    if shape["type"] == "sphere":
        return np.linalg.norm(points, axis=-1) - float(shape["radius"])
    if shape["type"] == "box":
        half = 0.5 * np.asarray(shape["size_xyz"], dtype=np.float64)
        q = np.abs(points) - half
        return np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(np.max(q, axis=-1), 0.0)
    half_length = 0.5 * float(shape["length"])
    delta = points.copy()
    delta[..., 2] -= np.clip(delta[..., 2], -half_length, half_length)
    return np.linalg.norm(delta, axis=-1) - float(shape["radius"])


@dataclass(frozen=True)
class TimedCollisionPlan:
    absolute_times_s: np.ndarray
    sphere_positions: np.ndarray
    sphere_radii: np.ndarray

    def __post_init__(self):
        times = np.asarray(self.absolute_times_s, dtype=np.float64)
        positions = np.asarray(self.sphere_positions, dtype=np.float64)
        radii = np.asarray(self.sphere_radii, dtype=np.float64)
        if times.ndim != 1 or len(times) < 2 or np.any(np.diff(times) <= 0.0):
            raise ValueError("collision plan times must be strictly increasing")
        if positions.ndim != 3 or positions.shape[0] != len(times) or positions.shape[-1] != 3:
            raise ValueError("sphere_positions must be [time,spheres,3]")
        if radii.shape != (positions.shape[1],) or not np.isfinite(positions).all():
            raise ValueError("collision sphere arrays are inconsistent")

    def sample(self, times_s: np.ndarray) -> np.ndarray:
        times_s = np.asarray(times_s, dtype=np.float64)
        if times_s.min() < self.absolute_times_s[0] or times_s.max() > self.absolute_times_s[-1]:
            raise ValueError("collision-plan sample is outside its duration")
        right = np.searchsorted(self.absolute_times_s, times_s, side="right")
        right = np.clip(right, 1, len(self.absolute_times_s) - 1)
        left = right - 1
        alpha = (
            (times_s - self.absolute_times_s[left])
            / (self.absolute_times_s[right] - self.absolute_times_s[left])
        )
        return (
            self.sphere_positions[left] * (1.0 - alpha[:, None, None])
            + self.sphere_positions[right] * alpha[:, None, None]
        )


@dataclass(frozen=True)
class TrajectoryRisk:
    safe: bool
    world_version: int
    minimum_clearance_m: float
    first_collision_unix_s: float | None
    checked_samples: int


class DynamicTrajectoryGuard:
    def __init__(
        self,
        *,
        check_dt_s: float = 0.02,
        covariance_sigma: float = 3.0,
        process_acceleration_std_m_s2: float = 0.01,
        minimum_clearance_m: float = 0.0,
    ) -> None:
        if check_dt_s <= 0.0:
            raise ValueError("check_dt_s must be positive")
        self.check_dt_s = float(check_dt_s)
        self.covariance_sigma = float(covariance_sigma)
        self.process_variance = float(process_acceleration_std_m_s2) ** 2
        self.minimum_clearance_m = float(minimum_clearance_m)

    def validate(
        self,
        plan: TimedCollisionPlan,
        world: DynamicWorldSnapshot,
        start_unix_s: float,
        end_unix_s: float,
    ) -> TrajectoryRisk:
        start = max(float(start_unix_s), float(plan.absolute_times_s[0]))
        end = min(float(end_unix_s), float(plan.absolute_times_s[-1]))
        if end < start:
            raise ValueError("requested validation interval is empty")
        if start * 1e9 < world.stamp_unix_ns or end * 1e9 > world.valid_until_unix_ns:
            return TrajectoryRisk(False, world.version, -math.inf, start, 0)
        count = max(2, int(math.ceil((end - start) / self.check_dt_s)) + 1)
        times = np.linspace(start, end, count)
        spheres = plan.sample(times)
        minimum = math.inf
        first_collision = None
        dt = times - world.stamp_unix_ns * 1e-9
        for item in world.objects:
            center = np.asarray(item.position) + dt[:, None] * np.asarray(item.linear_velocity)
            local = (spheres - center[:, None, :]) @ _rotation_xyzw(item.orientation_xyzw)
            distances = _local_sdf(local, item)
            distances -= _inflation(item, dt, self.covariance_sigma, self.process_variance)[:, None]
            clearance = distances - plan.sphere_radii[None, :]
            minimum = min(minimum, float(clearance.min()))
            collisions = np.any(clearance <= self.minimum_clearance_m, axis=1)
            if np.any(collisions):
                collision_time = float(times[np.flatnonzero(collisions)[0]])
                first_collision = collision_time if first_collision is None else min(first_collision, collision_time)
        return TrajectoryRisk(
            first_collision is None,
            world.version,
            minimum,
            first_collision,
            count,
        )


def collision_plan_from_result(result, trajectory_start_unix_s: float) -> TimedCollisionPlan:
    diagnostics = result.diagnostics
    times = np.asarray([point.time_from_start_s for point in result.points], dtype=np.float64)
    return TimedCollisionPlan(
        absolute_times_s=trajectory_start_unix_s + times,
        sphere_positions=np.asarray(diagnostics["collision_sphere_positions"], dtype=np.float64),
        sphere_radii=np.asarray(diagnostics["collision_sphere_radii"], dtype=np.float64),
    )


def splice_collision_plans(
    active: TimedCollisionPlan | None,
    new: TimedCollisionPlan,
    commit_start_unix_s: float,
    handoff_unix_s: float,
    prefix_dt_s: float,
) -> TimedCollisionPlan:
    if active is None:
        prefix_times = np.asarray([commit_start_unix_s, handoff_unix_s])
        prefix_positions = np.repeat(new.sphere_positions[:1], 2, axis=0)
        keep = new.absolute_times_s > handoff_unix_s + 1e-9
        return TimedCollisionPlan(
            absolute_times_s=np.concatenate((prefix_times, new.absolute_times_s[keep])),
            sphere_positions=np.concatenate(
                (prefix_positions, new.sphere_positions[keep]), axis=0
            ),
            sphere_radii=new.sphere_radii,
        )
    count = max(2, int(math.ceil((handoff_unix_s - commit_start_unix_s) / prefix_dt_s)) + 1)
    prefix_times = np.linspace(commit_start_unix_s, handoff_unix_s, count)
    prefix_positions = active.sample(prefix_times)
    keep = new.absolute_times_s > handoff_unix_s + 1e-9
    return TimedCollisionPlan(
        absolute_times_s=np.concatenate((prefix_times, new.absolute_times_s[keep])),
        sphere_positions=np.concatenate((prefix_positions, new.sphere_positions[keep]), axis=0),
        sphere_radii=new.sphere_radii,
    )
