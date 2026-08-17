"""Owned FollowJointTrajectory goal lifecycle for Phase 3."""

from __future__ import annotations

from typing import Callable

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory


class JtcHandoffManager:
    """Keep goal handles and correlate every callback with a monotonic plan ID."""

    def __init__(
        self,
        node,
        action_name: str,
        *,
        on_accepted: Callable[[int], None],
        on_terminal: Callable[[int, str], None],
    ) -> None:
        self._node = node
        self._client = ActionClient(node, FollowJointTrajectory, action_name)
        self._on_accepted = on_accepted
        self._on_terminal = on_terminal
        self._goal_handle = None
        self.active_plan_id: int | None = None
        self.pending_plan_id: int | None = None
        self.state = "IDLE"
        self.last_terminal_state: str | None = None
        self.last_result_error_code: int | None = None
        self.last_result_error_string: str | None = None

    @property
    def plan_id(self) -> int | None:
        """Currently owned, accepted JTC plan ID."""
        return self.active_plan_id

    def submit(self, plan_id: int, trajectory: JointTrajectory) -> bool:
        """Send without blocking; JTC acceptance atomically preempts old goal."""
        if not self._client.server_is_ready():
            self.state = "UNAVAILABLE"
            return False
        self.pending_plan_id = plan_id
        self.state = "SENDING"
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, captured=plan_id: self._goal_response(captured, completed)
        )
        return True

    def _goal_response(self, plan_id: int, future) -> None:
        try:
            handle = future.result()
        except Exception as error:
            if plan_id == self.pending_plan_id:
                self.pending_plan_id = None
                self.state = "ACTIVE" if self._goal_handle is not None else "SEND_ERROR"
                self.last_terminal_state = f"SEND_ERROR:{error}"
                self._on_terminal(plan_id, "SEND_ERROR")
            return
        if not handle.accepted:
            if plan_id == self.pending_plan_id:
                self.pending_plan_id = None
                self.state = "ACTIVE" if self._goal_handle is not None else "REJECTED"
                self.last_terminal_state = "REJECTED"
                self._on_terminal(plan_id, "REJECTED")
            return
        if plan_id != self.pending_plan_id:
            handle.cancel_goal_async()
            return
        self.pending_plan_id = None
        self._goal_handle = handle
        self.active_plan_id = plan_id
        self.state = "ACTIVE"
        self._on_accepted(plan_id)
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, captured=plan_id: self._result(captured, completed)
        )

    def _result(self, plan_id: int, future) -> None:
        try:
            response = future.result()
            status = response.status
            result = response.result
            error_code = getattr(result, "error_code", None)
            error_string = getattr(result, "error_string", None)
        except Exception:
            terminal = "RESULT_ERROR"
            error_code = None
            error_string = None
        else:
            terminal = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
            }.get(status, f"STATUS_{status}")
        if plan_id == self.active_plan_id:
            self.state = terminal
            self.last_terminal_state = terminal
            self.last_result_error_code = error_code
            self.last_result_error_string = error_string
            self._goal_handle = None
            self.active_plan_id = None
        self._on_terminal(plan_id, terminal)

    def cancel(self) -> bool:
        """Request controller-side cancellation of the owned active goal."""
        self.pending_plan_id = None
        if self._goal_handle is None:
            self.state = "STOPPED"
            return False
        self.state = "CANCELING"
        self._goal_handle.cancel_goal_async()
        return True

    def destroy(self) -> None:
        """Release action-client resources."""
        self._client.destroy()
