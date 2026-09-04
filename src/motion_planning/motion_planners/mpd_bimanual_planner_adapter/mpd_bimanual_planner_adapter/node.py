"""ROS2 action façade; worker IPC is intentionally injectable for testing."""
from __future__ import annotations

def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionServer
        from manipulation_planning_interfaces.action import PlanBimanualTrajectory
    except ImportError as error:
        raise RuntimeError("source ROS2 and manipulation_planning_interfaces before starting the adapter") from error

    class BimanualPlannerNode(Node):
        def __init__(self):
            super().__init__("mpd_bimanual_planner_adapter", namespace="marvin/mpd_bimanual")
            self._server = ActionServer(self, PlanBimanualTrajectory, "plan", self._execute)

        async def _execute(self, goal_handle):
            from .contract import validate_goal
            try:
                validate_goal(goal_handle.request)
            except ValueError as error:
                result = PlanBimanualTrajectory.Result()
                result.success = False
                result.error_code = "INVALID_GOAL"
                result.message = str(error)
                goal_handle.abort()
                return result
            result = PlanBimanualTrajectory.Result()
            result.success = False
            result.error_code = "WORKER_NOT_CONFIGURED"
            result.message = "Configure MPD worker IPC before execute:=true"
            goal_handle.abort()
            return result

    rclpy.init(args=args)
    node = BimanualPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
