#!/usr/bin/env python3
"""Publish one conservative trajectory and verify the EM-to-JTC closed loop."""

import sys
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [f"Joint{i}_L" for i in range(1, 8)]
TARGET = [0.0] * 7


class TrajectoryChainProbe(Node):
    def __init__(self):
        super().__init__("trajectory_chain_probe")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            JointTrajectory,
            "/action_sources/trajectory_test/joint_trajectory_goal",
            qos,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._status_subscription = self.create_subscription(
            GoalStatusArray,
            "/left_arm_controller/follow_joint_trajectory/_action/status",
            self._on_status,
            10,
        )
        self._positions = None
        self._saw_executing = False
        self._saw_succeeded = False
        self._published_at = None
        self._started_at = time.monotonic()
        self.exit_code = 1
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            "Gate ready: waiting for EM subscription, JTC status, and joint states"
        )

    def _on_joint_state(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            self._positions = [positions[name] for name in JOINT_NAMES]

    def _on_status(self, msg: GoalStatusArray):
        statuses = {entry.status for entry in msg.status_list}
        self._saw_executing |= GoalStatus.STATUS_EXECUTING in statuses
        self._saw_succeeded |= GoalStatus.STATUS_SUCCEEDED in statuses

    def _trajectory(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES
        for seconds, positions in (
            (0.5, [0.0] * 7),
            (1.5, [0.10] + [0.0] * 6),
            (2.5, TARGET),
        ):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = int((seconds % 1.0) * 1e9)
            trajectory.points.append(point)
        return trajectory

    def _tick(self):
        now = time.monotonic()
        if self._published_at is None:
            ready = (
                self._publisher.get_subscription_count() > 0
                and self._status_subscription.get_publisher_count() > 0
                and self._positions is not None
            )
            if ready:
                self._publisher.publish(self._trajectory())
                self._published_at = now
                self.get_logger().info(
                    "Published 3-point trajectory: Joint1_L 0.00 -> 0.10 -> 0.00 rad"
                )
            elif now - self._started_at > 10.0:
                self._finish(False, "startup timeout; EM subscription or joint states missing")
            return

        if self._saw_succeeded:
            max_error = max(abs(actual - target) for actual, target in zip(self._positions, TARGET))
            if self._saw_executing and max_error <= 0.02:
                self._finish(True, f"JTC succeeded; final max joint error={max_error:.6f} rad")
            else:
                self._finish(
                    False,
                    f"incomplete evidence: executing={self._saw_executing}, error={max_error:.6f} rad",
                )
        elif now - self._published_at > 8.0:
            self._finish(False, "trajectory timeout; JTC did not report SUCCEEDED")

    def _finish(self, passed: bool, detail: str):
        self.exit_code = 0 if passed else 1
        level = self.get_logger().info if passed else self.get_logger().error
        level(f"{'PASS' if passed else 'FAIL'} trajectory -> EM -> JTC: {detail}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TrajectoryChainProbe()
    try:
        rclpy.spin(node)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
