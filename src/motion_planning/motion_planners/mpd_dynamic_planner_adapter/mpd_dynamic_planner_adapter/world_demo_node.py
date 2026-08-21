"""Deterministic known-object observation source for fake-hardware validation."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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
        if self._scenario == "safe_far":
            position = [1.5, 1.5 + 0.05 * elapsed, 1.5]
            local_sdf = {"type": "box", "size_xyz": [0.20, 0.12, 0.30]}
            base_inflation_m = 0.01
            horizon_inflation_rate_m_s = 0.005
        elif self._scenario == "crossing":
            position = [0.55, -0.5 + 0.08 * elapsed, 0.45]
            local_sdf = {"type": "box", "size_xyz": [0.20, 0.12, 0.30]}
            base_inflation_m = 0.01
            horizon_inflation_rate_m_s = 0.005
        elif self._scenario == "to_drawer_crossing":
            position = [0.4 - 0.045 * elapsed, 0.40, 0.38]
            local_sdf = {"type": "box", "size_xyz": [0.16, 0.12, 0.18]}
            base_inflation_m = 0.03
            horizon_inflation_rate_m_s = 0.02
        elif self._scenario == "to_drawer_bridge_crossing":
            # A repeatable crossing that remains solvable long enough to
            # exercise multiple moving-state trajectory replacements.
            if elapsed < 15.0:
                position = [0.32 - 0.09 * elapsed, 0.40, 0.38]
            else:
                position = [-1.03 + 0.09 *  (elapsed - 15.0), 0.40, 0.38]
            local_sdf = {"type": "box", "size_xyz": [0.14, 0.10, 0.16]}
            base_inflation_m = 0.02
            horizon_inflation_rate_m_s = 0.01
        else:
            self.get_logger().error(f"unknown scenario {self._scenario!r}")
            return
        message = String()
        message.data = json.dumps(
            {
                "frame_id": "fr3_link0",
                "stamp_unix_ns": time.time_ns(),
                "objects": [
                    {
                        "id": "demo-box",
                        "local_sdf": local_sdf,
                        "position": position,
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "position_covariance_3x3": [
                            0.0001,
                            0.0,
                            0.0,
                            0.0,
                            0.0001,
                            0.0,
                            0.0,
                            0.0,
                            0.0001,
                        ],
                        "inflation_mode": "linear",
                        "base_inflation_m": base_inflation_m,
                        "horizon_inflation_rate_m_s": horizon_inflation_rate_m_s,
                    }
                ],
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
