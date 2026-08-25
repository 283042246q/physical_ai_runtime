"""ROS 2 Phase-5 candidate-specific space-time replanner entrypoint."""

from __future__ import annotations

import rclpy

from .backend import SpaceTimeMpdGlobalTrajectoryBackend
from .replan_node import MpdDynamicReplanNode


class MpdSpaceTimeReplanNode(MpdDynamicReplanNode):
    """Reuse the Phase-4 handoff/guard loop with a strict Phase-5 backend."""

    _default_clearance_score_mode = "mean_cvar"

    def __init__(self) -> None:
        # No planning callback can run before construction returns and spin starts,
        # so replacing the socket backend here leaves the Phase-4 node untouched.
        super().__init__()
        self.declare_parameter("timing_mode", "phase5_joint")
        value = lambda name: self.get_parameter(name).value
        self._backend = SpaceTimeMpdGlobalTrajectoryBackend(
            str(value("socket_path")),
            scene_id=str(value("scene_id")),
            seed=int(value("seed")),
            timeout_s=float(value("worker_timeout_s")),
            expected_timing_mode=str(value("timing_mode")),
        )
        self._backend_ready = False
        self.get_logger().info(
            f"Phase-5 space-time backend selected ({value('timing_mode')})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MpdSpaceTimeReplanNode()
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
