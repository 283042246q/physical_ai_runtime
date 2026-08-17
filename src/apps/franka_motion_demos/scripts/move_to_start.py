#!/usr/bin/env python3
"""Move the FR3 to Franka's start configuration through EM and effort JTC."""

from __future__ import annotations

import math
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
Q_GOAL_DEFAULT = [
    0.0,
    -math.pi / 4.0,
    0.0,
    -3.0 * math.pi / 4.0,
    0.0,
    math.pi / 2.0,
    math.pi / 4.0,
]
TARGET_ACTION = "/action_sources/trajectory_test/arm/joint_trajectory"


def _quintic(s: float) -> tuple[float, float, float]:
    s = float(np.clip(s, 0.0, 1.0))
    return (
        10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5,
        30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4,
        60.0 * s - 180.0 * s**2 + 120.0 * s**3,
    )


class MoveToStart(Node):
    def __init__(self) -> None:
        super().__init__("move_to_start")
        self.declare_parameter("q_goal", Q_GOAL_DEFAULT)
        self.declare_parameter("duration_s", 10.0)
        self.declare_parameter("num_points", 200)
        self.declare_parameter("wait_for_em_s", 20.0)
        self.declare_parameter("wait_for_result_s", 20.0)
        self.declare_parameter("goal_tolerance_rad", 0.02)
        self.declare_parameter("settle_samples", 10)

        self._goal = np.asarray(
            [float(value) for value in self.get_parameter("q_goal").value],
            dtype=np.float64,
        )
        self._duration = float(self.get_parameter("duration_s").value)
        self._num_points = int(self.get_parameter("num_points").value)
        self._wait_for_em = float(self.get_parameter("wait_for_em_s").value)
        self._wait_for_result = float(
            self.get_parameter("wait_for_result_s").value
        )
        self._tolerance = float(
            self.get_parameter("goal_tolerance_rad").value
        )
        self._settle_samples = int(
            self.get_parameter("settle_samples").value
        )
        if len(self._goal) != len(FR3_JOINTS):
            raise ValueError("q_goal must contain seven joints")
        if self._duration <= 0.0 or self._num_points < 2:
            raise ValueError("duration_s > 0 and num_points >= 2 required")
        if self._tolerance <= 0.0 or self._settle_samples < 1:
            raise ValueError(
                "goal_tolerance_rad must be positive and settle_samples >= 1"
            )

        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            TARGET_ACTION,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._positions: np.ndarray | None = None
        self._published_at: float | None = None
        self._saw_executing = False
        self._saw_succeeded = False
        self._settled_count = 0
        self._started_at = time.monotonic()
        self.exit_code = 1
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "Waiting to route a smooth start-pose trajectory through EM -> JTC"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in FR3_JOINTS):
            self._positions = np.asarray(
                [positions[name] for name in FR3_JOINTS],
                dtype=np.float64,
            )

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
        self._saw_succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if not self._saw_succeeded:
            self._finish(
                False,
                f"trajectory ended with status={wrapped.status}, "
                f"error_code={result.error_code}: {result.error_string}",
            )

    def _max_error(self) -> float:
        assert self._positions is not None
        return float(np.max(np.abs(self._positions - self._goal)))

    def _trajectory(self) -> JointTrajectory:
        assert self._positions is not None
        start = self._positions.copy()
        delta = self._goal - start

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(FR3_JOINTS)
        for time_s in np.linspace(0.0, self._duration, self._num_points):
            scale, d1, d2 = _quintic(time_s / self._duration)
            point = JointTrajectoryPoint()
            point.positions = (start + delta * scale).tolist()
            point.velocities = (delta * d1 / self._duration).tolist()
            point.accelerations = (
                delta * d2 / (self._duration * self._duration)
            ).tolist()
            nanoseconds = round(float(time_s) * 1e9)
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = (
                nanoseconds % 1_000_000_000
            )
            trajectory.points.append(point)

        trajectory.points[0].positions = start.tolist()
        trajectory.points[0].velocities = [0.0] * len(FR3_JOINTS)
        trajectory.points[0].accelerations = [0.0] * len(FR3_JOINTS)
        trajectory.points[-1].positions = self._goal.tolist()
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
                error = self._max_error()
                if error <= self._tolerance:
                    self._finish(True, "already at the start configuration")
                    return
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = self._trajectory()
                self._trajectory_client.send_goal_async(
                    goal,
                    feedback_callback=self._on_feedback,
                ).add_done_callback(self._on_goal_response)
                self._published_at = now
                self.get_logger().info(
                    "Submitted smooth start-pose trajectory through EM: "
                    f"duration={self._duration:.1f}s points={self._num_points} "
                    f"max_delta={error:.3f}rad"
                )
            elif now - self._started_at > self._wait_for_em:
                self._finish(False, "startup timeout")
            return

        error = self._max_error() if self._positions is not None else math.inf
        self._settled_count = (
            self._settled_count + 1
            if self._saw_succeeded and error <= self._tolerance
            else 0
        )
        if self._settled_count >= self._settle_samples:
            self._finish(
                True,
                "JTC reached start configuration; "
                f"max error={error:.6f} rad",
            )
        elif now - self._published_at > self._wait_for_result:
            self._finish(
                False,
                "JTC start-pose timeout: "
                f"executing={self._saw_executing} "
                f"succeeded={self._saw_succeeded} max_error={error:.6f} rad",
            )

    def _finish(self, passed: bool, detail: str) -> None:
        self.exit_code = 0 if passed else 1
        log = self.get_logger().info if passed else self.get_logger().error
        log(f"{'PASS' if passed else 'FAIL'}: {detail}")
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = MoveToStart()
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
