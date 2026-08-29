"""Deterministic known-object observation source for fake-hardware validation."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


_POSITION_COVARIANCE_3X3 = [
    0.0001,
    0.0,
    0.0,
    0.0,
    0.0001,
    0.0,
    0.0,
    0.0,
    0.0001,
]

# (object id, path-crossing xyz, unit direction xy, crossing time, speed)
# Each xy direction is perpendicular to the measured local tangent of a
# successful ToDrawer end-effector path at the corresponding crossing point.
_TO_DRAWER_CROSSING_SPECS = (
    (
        "demo-box-crossing-1",
        (-0.68831415, -1.22503140, 0.65347225),
        (0.61993897, 0.78465003, 0.0),
        13.5,
        0.18,
    ),
    (
        "demo-box-crossing-2",
        (-0.02146948, 0.40876295, 0.46108561),
        (0.99858189, -0.05323737, 0.0),
        12.5,
        0.18,
    ),
    (
        "demo-box-crossing-3",
        (-0.2, -0.16, 0.5),
        (0.0, 0.0, 1.0),
        14.5,
        0.18,
    ),
)


def _box_observation(
    object_id: str,
    position: list[float],
    *,
    size_xyz: list[float],
    base_inflation_m: float,
    horizon_inflation_rate_m_s: float,
) -> dict:
    return {
        "id": object_id,
        "local_sdf": {"type": "box", "size_xyz": size_xyz},
        "position": position,
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "position_covariance_3x3": _POSITION_COVARIANCE_3X3,
        "inflation_mode": "linear",
        "base_inflation_m": base_inflation_m,
        "horizon_inflation_rate_m_s": horizon_inflation_rate_m_s,
    }


def _scenario_objects(scenario: str, elapsed: float) -> list[dict]:
    if scenario == "safe_far":
        return [
            _box_observation(
                "demo-box",
                [1.5, 1.5 + 0.05 * elapsed, 1.5],
                size_xyz=[0.20, 0.12, 0.30],
                base_inflation_m=0.01,
                horizon_inflation_rate_m_s=0.005,
            )
        ]
    if scenario == "crossing":
        return [
            _box_observation(
                "demo-box",
                [0.55, -0.5 + 0.08 * elapsed, 0.45],
                size_xyz=[0.20, 0.12, 0.30],
                base_inflation_m=0.01,
                horizon_inflation_rate_m_s=0.005,
            )
        ]
    if scenario == "to_drawer_crossing":
        if elapsed < 10.0:
            position = [0.32 - 0.2 * elapsed, 0.40, 0.38]
        else:
            position = [
                0.32 - 0.2 * 10.0 + 0.06 * (elapsed - 10.0),
                0.40,
                0.38,
            ]
        return [
            _box_observation(
                "demo-box",
                position,
                size_xyz=[0.16, 0.12, 0.18],
                base_inflation_m=0.03,
                horizon_inflation_rate_m_s=0.02,
            )
        ]
        #return [
        #    _box_observation(
        #        "demo-box",
        #        [0.4 - 0.045 * elapsed, 0.40, 0.38],
        #        size_xyz=[0.16, 0.12, 0.18],
        #        base_inflation_m=0.03,
        #        horizon_inflation_rate_m_s=0.02,
        #    )
        #]
    if scenario == "to_drawer_bridge_crossing":
        # Previous single reversing obstacle retained for easy comparison:
        # if elapsed < 7.0:
        #     position = [0.32 - 0.2 * elapsed, 0.40, 0.38]
        # else:
        #     position = [
        #         0.32 - 0.2 * 7.0 + 0.06 * (elapsed - 7.0), 0.40, 0.38
        #     ]
        objects = []
        for object_id, anchor, direction, crossing_time, speed in (
            _TO_DRAWER_CROSSING_SPECS
        ):
            displacement = speed * (elapsed - crossing_time)
            position = [
                anchor[0] + direction[0] * displacement,
                anchor[1] + direction[1] * displacement,
                anchor[2] + direction[2] * displacement,
            ]
            objects.append(
                _box_observation(
                    object_id,
                    position,
                    size_xyz=[0.12, 0.12, 0.16],
                    base_inflation_m=0.02,
                    horizon_inflation_rate_m_s=0.01,
                )
            )
        return objects
    raise ValueError(f"unknown scenario {scenario!r}")


def _scenario_file(path: str | Path) -> dict:
    source = Path(path).expanduser().resolve()
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("dynamic scenario file must contain a JSON object")
    if payload.get("schema") != "mpd_todrawer_dynamic_scenario":
        raise ValueError("dynamic scenario file has an unsupported schema")
    if payload.get("schema_version") != 1:
        raise ValueError("dynamic scenario schema_version must be 1")
    if payload.get("frame_id") != "fr3_link0":
        raise ValueError("dynamic scenario frame_id must be 'fr3_link0'")
    objects = payload.get("objects")
    if not isinstance(objects, list) or not 1 <= len(objects) <= 16:
        raise ValueError("dynamic scenario objects must contain 1..16 entries")

    normalized = []
    seen = set()
    for item in objects:
        if not isinstance(item, dict):
            raise ValueError("dynamic scenario objects must be JSON objects")
        object_id = item.get("id")
        if not isinstance(object_id, str) or not object_id or object_id in seen:
            raise ValueError("dynamic scenario object ids must be unique non-empty strings")
        seen.add(object_id)

        def vector(name: str, size: int) -> list[float]:
            value = item.get(name)
            if (
                not isinstance(value, list)
                or len(value) != size
                or any(isinstance(entry, bool) or not math.isfinite(float(entry)) for entry in value)
            ):
                raise ValueError(f"dynamic scenario {object_id}.{name} must have {size} finite values")
            return [float(entry) for entry in value]

        anchor = vector("anchor_position", 3)
        direction = vector("direction", 3)
        direction_norm = math.sqrt(sum(value * value for value in direction))
        if direction_norm <= 1.0e-9:
            raise ValueError(f"dynamic scenario {object_id}.direction must be non-zero")
        direction = [value / direction_norm for value in direction]
        crossing_time = float(item.get("crossing_time_s", math.nan))
        speed = float(item.get("speed_m_s", math.nan))
        if not math.isfinite(crossing_time) or not math.isfinite(speed) or speed < 0.0:
            raise ValueError(
                f"dynamic scenario {object_id} crossing_time_s/speed_m_s are invalid"
            )
        local_sdf = item.get("local_sdf")
        if not isinstance(local_sdf, dict) or local_sdf.get("type") not in {
            "box",
            "sphere",
            "capsule",
        }:
            raise ValueError(f"dynamic scenario {object_id}.local_sdf is invalid")
        orientation = vector("orientation_xyzw", 4)
        covariance = vector("position_covariance_3x3", 9)
        inflation = item.get("inflation")
        if not isinstance(inflation, dict) or inflation.get("mode") not in {
            "linear",
            "covariance",
        }:
            raise ValueError(f"dynamic scenario {object_id}.inflation is invalid")
        base = float(inflation.get("base_m", math.nan))
        rate = float(inflation.get("horizon_rate_m_s", math.nan))
        if not all(math.isfinite(value) and value >= 0.0 for value in (base, rate)):
            raise ValueError(f"dynamic scenario {object_id}.inflation values are invalid")
        normalized.append(
            {
                "id": object_id,
                "local_sdf": local_sdf,
                "anchor_position": anchor,
                "direction": direction,
                "crossing_time_s": crossing_time,
                "speed_m_s": speed,
                "orientation_xyzw": orientation,
                "position_covariance_3x3": covariance,
                "inflation": {
                    "mode": str(inflation["mode"]),
                    "base_m": base,
                    "horizon_rate_m_s": rate,
                },
            }
        )
    return {**payload, "objects": normalized}


def _scenario_file_objects(payload: dict, elapsed: float) -> list[dict]:
    objects = []
    for item in payload["objects"]:
        displacement = item["speed_m_s"] * (elapsed - item["crossing_time_s"])
        position = [
            anchor + direction * displacement
            for anchor, direction in zip(item["anchor_position"], item["direction"])
        ]
        inflation = item["inflation"]
        objects.append(
            {
                "id": item["id"],
                "local_sdf": item["local_sdf"],
                "position": position,
                "orientation_xyzw": item["orientation_xyzw"],
                "position_covariance_3x3": item["position_covariance_3x3"],
                "inflation_mode": inflation["mode"],
                "base_inflation_m": inflation["base_m"],
                "horizon_inflation_rate_m_s": inflation["horizon_rate_m_s"],
            }
        )
    return objects


class DynamicWorldDemoNode(Node):
    def __init__(self) -> None:
        super().__init__("mpd_dynamic_world_demo")
        self.declare_parameter("topic", "/mpd/dynamic_world_observations")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("scenario", "safe_far")
        self.declare_parameter("scenario_file", "")
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._scenario = str(self.get_parameter("scenario").value)
        scenario_file = str(self.get_parameter("scenario_file").value)
        self._scenario_payload = _scenario_file(scenario_file) if scenario_file else None
        self._frame_id = (
            str(self._scenario_payload["frame_id"])
            if self._scenario_payload is not None
            else "fr3_link0"
        )
        self._publisher = self.create_publisher(
            String, str(self.get_parameter("topic").value), 1
        )
        self._started = time.time()
        self.create_timer(1.0 / rate, self._publish)

    def _publish(self) -> None:
        elapsed = time.time() - self._started
        try:
            objects = (
                _scenario_objects(self._scenario, elapsed)
                if self._scenario_payload is None
                else _scenario_file_objects(self._scenario_payload, elapsed)
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        message = String()
        message.data = json.dumps(
            {
                "frame_id": self._frame_id,
                "stamp_unix_ns": time.time_ns(),
                "objects": objects,
            },
            separators=(",", ":"),
        )
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicWorldDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
