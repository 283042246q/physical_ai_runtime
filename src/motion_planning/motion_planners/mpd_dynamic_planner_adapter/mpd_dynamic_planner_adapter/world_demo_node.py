"""Deterministic known-object observation source for fake-hardware validation."""

from __future__ import annotations

import json
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
    #(
    #    "demo-box-crossing-1",
    #    (-0.68831415, -1.22503140, 0.65347225),
    #    (0.61993897, 0.78465003, 0.0),
    #    13.5,
    #    0.18,
    #),
    (
        "demo-box-crossing-2",
        (-0.02146948, 0.30876295, 0.46108561),
        (0.99858189, -0.05323737, 0.0),
        8.5,
        0.18,
    ),
    #(
    #    "demo-box-crossing-3",
    #    (-0.2, -0.16, 0.5),
    #    (0.0, 0.0, 1.0),
    #    14.5,
    #    0.18,
    #),
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


class DynamicWorldDemoNode(Node):
    def __init__(self) -> None:
        super().__init__("mpd_dynamic_world_demo")
        self.declare_parameter("topic", "/mpd/dynamic_world_observations")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("scenario", "safe_far")
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._scenario = str(self.get_parameter("scenario").value)
        self._publisher = self.create_publisher(
            String, str(self.get_parameter("topic").value), 1
        )
        self._started = time.time()
        self.create_timer(1.0 / rate, self._publish)

    def _publish(self) -> None:
        elapsed = time.time() - self._started
        try:
            objects = _scenario_objects(self._scenario, elapsed)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        message = String()
        message.data = json.dumps(
            {
                "frame_id": "fr3_link0",
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
