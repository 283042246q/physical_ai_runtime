"""Independent cancel-all interface for the configured JTC action server."""

from __future__ import annotations

import json

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def cancel_all_request() -> CancelGoal.Request:
    """All-zero GoalInfo requests cancellation of every goal on the server."""
    return CancelGoal.Request()


class JtcSafeStopNode(Node):
    """Remain usable when the planner worker or replanner is unavailable."""

    def __init__(self) -> None:
        super().__init__("mpd_jtc_safe_stop")
        action_name = str(
            self.declare_parameter(
                "jtc_action_name", "/franka_arm_jtc/follow_joint_trajectory"
            ).value
        ).rstrip("/")
        self._cancel_client = self.create_client(
            CancelGoal, f"{action_name}/_action/cancel_goal"
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._stop_latch = self.create_publisher(Bool, "/mpd/emergency_stop", latched_qos)
        self._status = self.create_publisher(String, "~/status", latched_qos)
        self.create_service(Trigger, "~/stop", self._on_stop)

    def _on_stop(self, request, response):
        self._stop_latch.publish(Bool(data=True))
        if not self._cancel_client.service_is_ready():
            response.success = False
            response.message = "JTC cancel service unavailable"
            return response
        future = self._cancel_client.call_async(cancel_all_request())
        future.add_done_callback(self._on_cancel_response)
        response.success = True
        response.message = "JTC cancel-all request dispatched"
        return response

    def _on_cancel_response(self, future) -> None:
        payload = {"operation": "cancel_all"}
        try:
            result = future.result()
            payload.update(
                return_code=int(result.return_code),
                goals_canceling=len(result.goals_canceling),
            )
        except Exception as error:
            payload["error"] = f"{type(error).__name__}: {error}"
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._status.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JtcSafeStopNode()
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
