from types import SimpleNamespace

import rclpy

from mpd_dynamic_planner_adapter.replan_node import MpdDynamicReplanNode


def test_dynamic_replanner_starts_without_target_or_world():
    rclpy.init()
    node = MpdDynamicReplanNode()
    try:
        assert node._target is None
        assert node._world_manager.snapshot is None
        assert node._plan_only
        assert not node._goal_reached
        assert node._guard_lookahead_s == 3.0
        assert node._backend.client.socket_path.name == "mpd-dynamic-runtime.sock"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_success_keeps_target_and_enters_protected_terminal_hold():
    rclpy.init()
    node = MpdDynamicReplanNode()
    try:
        target = object()
        active_plan = object()
        collision_plan = object()
        node._target = target
        node._active_plan = active_plan
        node._active_collision_plan = collision_plan
        node._active_plan_id = 7
        node._execution = SimpleNamespace(plan_id=None, pending_plan_id=None)

        node._on_goal_terminal(7, "SUCCEEDED")

        assert node._target is target
        assert node._active_plan is active_plan
        assert node._active_collision_plan is collision_plan
        assert node._goal_reached
        assert node._last_switch_decision == "goal_reached_terminal_hold"
    finally:
        node._execution = None
        node.destroy_node()
        rclpy.shutdown()


def test_rejected_replacement_does_not_discard_terminal_hold():
    rclpy.init()
    node = MpdDynamicReplanNode()
    try:
        active_plan = object()
        collision_plan = object()
        node._active_plan = active_plan
        node._active_collision_plan = collision_plan
        node._active_plan_id = 7
        node._goal_reached = True
        node._execution = SimpleNamespace(plan_id=None, pending_plan_id=None)

        node._on_goal_terminal(8, "REJECTED")

        assert node._active_plan is active_plan
        assert node._active_collision_plan is collision_plan
        assert node._active_plan_id == 7
        assert node._goal_reached
    finally:
        node._execution = None
        node.destroy_node()
        rclpy.shutdown()
