#!/usr/bin/env python3
"""Send a smooth current-relative out-and-back trajectory through EM."""

from __future__ import annotations

import sys
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


FR3_JOINTS = [f"fr3_joint{index}" for index in range(1, 8)]


def _quintic(s: float) -> tuple[float, float, float]:
    s = float(np.clip(s, 0.0, 1.0))
    return (
        10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5,
        30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4,
        60.0 * s - 180.0 * s**2 + 120.0 * s**3,
    )


class SmoothTrajectory(Node):
    def __init__(self) -> None:
        super().__init__("smooth_trajectory")
        self.declare_parameter("joint_index", 3)
        self.declare_parameter("amplitude_rad", 0.25)
        self.declare_parameter("duration_s", 4.0)
        self.declare_parameter("num_points", 80)
        self.declare_parameter("wait_for_em_s", 15.0)
        self.declare_parameter("wait_for_result_s", 20.0)

        self._joint_index = int(self.get_parameter("joint_index").value)
        self._amplitude = float(self.get_parameter("amplitude_rad").value)
        self._duration = float(self.get_parameter("duration_s").value)
        self._num_points = int(self.get_parameter("num_points").value)
        self._wait_for_em = float(self.get_parameter("wait_for_em_s").value)
        self._wait_for_result = float(
            self.get_parameter("wait_for_result_s").value
        )
        if not 0 <= self._joint_index < len(FR3_JOINTS):
            raise ValueError("joint_index must be in [0, 6]")
        if self._duration <= 0.0 or self._num_points < 3:
            raise ValueError("duration_s > 0 and num_points >= 3 required")

        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/action_sources/trajectory_test/arm/joint_trajectory",
        )
        self.create_subscription(
            JointState,
            "/franka/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._positions: list[float] | None = None
        self._published_at: float | None = None
        self._saw_executing = False
        self._saw_succeeded = False
        self._started_at = time.monotonic()
        self.exit_code = 1
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            "Waiting for server-owned EM, effort JTC, and joint states"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in FR3_JOINTS):
            self._positions = [positions[name] for name in FR3_JOINTS]

    def _on_feedback(self, _feedback) -> None:
        self._saw_executing = True

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish(False, f"trajectory admission failed: {exc}")
            return
        if not goal_handle.accepted:
            self._finish(False, "EM rejected the trajectory")
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish(False, f"trajectory result failed: {exc}")
            return
        result = wrapped.result
        if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self._saw_succeeded = True
            return
        self._finish(
            False,
            f"trajectory ended with status={wrapped.status}, "
            f"error_code={result.error_code}: {result.error_string}",
        )

    def _trajectory(self) -> JointTrajectory:
        assert self._positions is not None
        start = np.asarray(self._positions, dtype=np.float64)
        delta = np.zeros(len(FR3_JOINTS), dtype=np.float64)
        delta[self._joint_index] = self._amplitude
        half = self._duration * 0.5

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(FR3_JOINTS)
        for time_s in np.linspace(
            0.0, self._duration, self._num_points
        ):
            if time_s <= half:
                scale, d1, d2 = _quintic(time_s / half)
                velocity_scale = d1 / half
                acceleration_scale = d2 / (half * half)
            else:
                scale_forward, d1, d2 = _quintic(
                    (time_s - half) / half
                )
                scale = 1.0 - scale_forward
                velocity_scale = -d1 / half
                acceleration_scale = -d2 / (half * half)

            point = JointTrajectoryPoint()
            point.positions = (start + delta * scale).tolist()
            point.velocities = (delta * velocity_scale).tolist()
            point.accelerations = (delta * acceleration_scale).tolist()
            nanoseconds = round(float(time_s) * 1e9)
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = (
                nanoseconds % 1_000_000_000
            )
            trajectory.points.append(point)
        trajectory.points[-1].positions = start.tolist()
        trajectory.points[-1].velocities = [0.0] * len(FR3_JOINTS)
        trajectory.points[-1].accelerations = [0.0] * len(FR3_JOINTS)
        return trajectory

    def _tick(self) -> None:
        now = time.monotonic()
        if self._published_at is None:
            ready = (
                self._trajectory_client.server_is_ready()
                and self._positions is not None
            )
            if ready:
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = self._trajectory()
                self._trajectory_client.send_goal_async(
                    goal,
                    feedback_callback=self._on_feedback,
                ).add_done_callback(self._on_goal_response)
                self._published_at = now
                self.get_logger().info(
                    "Submitted smooth out-and-back trajectory: "
                    f"{FR3_JOINTS[self._joint_index]} "
                    f"+{self._amplitude:.3f} rad"
                )
            elif now - self._started_at > self._wait_for_em:
                self._finish(False, "startup timeout")
            return

        if self._saw_succeeded:
            self._finish(
                self._saw_executing,
                "JTC reported SUCCEEDED after smooth trajectory",
            )
        elif now - self._published_at > self._wait_for_result:
            self._finish(False, "JTC result timeout")

    def _finish(self, passed: bool, detail: str) -> None:
        self.exit_code = 0 if passed else 1
        log = self.get_logger().info if passed else self.get_logger().error
        log(f"{'PASS' if passed else 'FAIL'}: {detail}")
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = SmoothTrajectory()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
