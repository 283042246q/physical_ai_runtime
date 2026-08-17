import rclpy

from mpd_planner_adapter.replan_node import MpdReplanNode


def test_replanner_starts_without_an_implicit_target():
    rclpy.init()
    node = None
    try:
        node = MpdReplanNode()
        assert node._target is None
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
