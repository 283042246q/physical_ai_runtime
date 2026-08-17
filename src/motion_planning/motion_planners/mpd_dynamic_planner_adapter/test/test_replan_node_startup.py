import rclpy

from mpd_dynamic_planner_adapter.replan_node import MpdDynamicReplanNode


def test_dynamic_replanner_starts_without_target_or_world():
    rclpy.init()
    node = MpdDynamicReplanNode()
    try:
        assert node._target is None
        assert node._world_manager.snapshot is None
        assert node._plan_only
        assert node._backend.client.socket_path.name == "mpd-dynamic-runtime.sock"
    finally:
        node.destroy_node()
        rclpy.shutdown()
