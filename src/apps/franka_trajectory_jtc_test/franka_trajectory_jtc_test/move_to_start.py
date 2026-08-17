#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Smooth move to Franka's official start pose via EM → effort JTC.

Goal matches ``MoveToStartExampleController`` /
``PTPMotionExampleNode`` (Franka ros2 examples):

  q_goal = {0, -π/4, 0, -3π/4, 0, π/2, π/4}

Unlike those examples (online MotionGenerator / PTP action), this script
emits one full ``JointTrajectory`` to the distributed executor:

  ros2 launch franka_trajectory_jtc_test trajectory_executor.launch.py
  ros2 run franka_trajectory_jtc_test move_to_start.py

Duration follows the MotionGenerator idea: ``speed_factor`` scales the
per-joint ``dq_max`` used to size a synchronized quintic segment.
"""

from __future__ import annotations

import math
import sys
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

FR3_JOINTS = [
    'fr3_joint1',
    'fr3_joint2',
    'fr3_joint3',
    'fr3_joint4',
    'fr3_joint5',
    'fr3_joint6',
    'fr3_joint7',
]

# Franka MoveToStartExampleController / PTPMotionExampleNode home pose.
#   Q_GOAL_DEFAULT = []
#   0.0,
#   -math.pi / 4.0,
#   0.0,
#   -3.0 * math.pi / 4.0,
#   0.0,
#   math.pi / 2.0,
#   math.pi / 4.0,
#   ]
Q_GOAL_DEFAULT = [0.7255,-0.1735,-0.4638,-2.9480,-0.3986,2.5254,-1.6282]

# MotionGenerator defaults (rad/s), scaled by speed_factor.
DQ_MAX = [2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]


def _quintic(s: float) -> tuple[float, float]:
    """Quintic blend s in [0,1] → (position_scale, d(scale)/ds)."""
    s = min(max(s, 0.0), 1.0)
    pos = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    dpos_ds = 30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4
    return pos, dpos_ds


def _estimate_duration_s(
    q0: list[float], q_goal: list[float], speed_factor: float
) -> float:
    """Synchronized duration from max |Δq_i| / (speed_factor * dq_max_i)."""
    speed_factor = min(max(speed_factor, 1e-3), 1.0)
    t_needed = 0.0
    for i, (a, b, dq_max) in enumerate(zip(q0, q_goal, DQ_MAX, strict=True)):
        delta = abs(b - a)
        t_needed = max(t_needed, delta / (speed_factor * dq_max))
    # Quintic peak speed > average; pad slightly so we stay under dq_max.
    return max(2.0, 1.35 * t_needed)


class MoveToStart(Node):
    def __init__(self) -> None:
        super().__init__('move_to_start')
        self.declare_parameter('joint_names', FR3_JOINTS)
        self.declare_parameter('joint_state_topic', '/franka/joint_states')
        self.declare_parameter(
            'goal_topic',
            '/action_sources/trajectory_test/joint_trajectory_goal',
        )
        self.declare_parameter(
            'jtc_status_topic',
            '/fr3_arm_controller/follow_joint_trajectory/_action/status',
        )
        self.declare_parameter('q_goal', Q_GOAL_DEFAULT)
        # 0 → auto from speed_factor (MotionGenerator-style).
        self.declare_parameter('duration_s', 0.0)
        self.declare_parameter('speed_factor', 0.2)
        self.declare_parameter('num_points', 100)
        self.declare_parameter('wait_for_em_s', 15.0)
        self.declare_parameter('wait_for_result_s', 60.0)
        self.declare_parameter('goal_tolerance_rad', 0.02)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.q_goal = [float(x) for x in self.get_parameter('q_goal').value]
        self.duration_s_param = float(self.get_parameter('duration_s').value)
        self.speed_factor = float(self.get_parameter('speed_factor').value)
        self.num_points = int(self.get_parameter('num_points').value)
        self.wait_for_em_s = float(self.get_parameter('wait_for_em_s').value)
        self.wait_for_result_s = float(
            self.get_parameter('wait_for_result_s').value
        )
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance_rad').value
        )

        if len(self.q_goal) != len(self.joint_names):
            raise ValueError('q_goal size must match joint_names')
        if self.num_points < 3:
            raise ValueError('num_points >= 3 required')
        if not (0.0 < self.speed_factor <= 1.0):
            raise ValueError('speed_factor must be in (0, 1]')

        goal_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter('goal_topic').value),
            goal_qos,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._status_sub = self.create_subscription(
            GoalStatusArray,
            str(self.get_parameter('jtc_status_topic').value),
            self._on_status,
            10,
        )

        self.q: list[float] | None = None
        self._published = False
        self._published_at: float | None = None
        self._duration_s = 0.0
        self._saw_executing = False
        self._saw_succeeded = False
        self.exit_code = 1
        self._started = time.monotonic()
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'move_to_start: waiting for joint states / EM / JTC '
            f'(q_goal={self.q_goal})'
        )

    def _on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self.joint_names):
            self.q = [float(positions[name]) for name in self.joint_names]

    def _on_status(self, msg: GoalStatusArray) -> None:
        statuses = {entry.status for entry in msg.status_list}
        self._saw_executing |= GoalStatus.STATUS_EXECUTING in statuses
        self._saw_succeeded |= GoalStatus.STATUS_SUCCEEDED in statuses

    def _build_trajectory(self) -> JointTrajectory:
        assert self.q is not None
        q0 = list(self.q)
        q1 = list(self.q_goal)
        delta = [b - a for a, b in zip(q0, q1, strict=True)]

        if self.duration_s_param > 0.0:
            T = self.duration_s_param
        else:
            T = _estimate_duration_s(q0, q1, self.speed_factor)
        self._duration_s = T
        n = self.num_points

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(self.joint_names)

        for i in range(n):
            t = T * i / (n - 1)
            s = t / T if T > 0.0 else 1.0
            scale, dscale_ds = _quintic(s)
            ds_dt = dscale_ds / T if T > 0.0 else 0.0

            point = JointTrajectoryPoint()
            point.positions = [a + d * scale for a, d in zip(q0, delta, strict=True)]
            point.velocities = [d * ds_dt for d in delta]
            nanoseconds = int(round(t * 1e9))
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = nanoseconds % 1_000_000_000
            traj.points.append(point)

        # Zero terminal velocity (quintic already ~0; enforce exactly).
        traj.points[-1].velocities = [0.0] * len(self.joint_names)
        return traj

    def _tick(self) -> None:
        now = time.monotonic()
        if not self._published:
            if now - self._started > self.wait_for_em_s:
                self.get_logger().error(
                    'Timeout waiting for EM / joint states / JTC status'
                )
                self.exit_code = 1
                rclpy.shutdown()
                return

            ready = (
                self.publisher.get_subscription_count() > 0
                and self._status_sub.get_publisher_count() > 0
                and self.q is not None
            )
            if not ready:
                return

            max_err = max(
                abs(a - b) for a, b in zip(self.q, self.q_goal, strict=True)
            )
            if max_err <= self.goal_tolerance:
                self.get_logger().info(
                    f'Already at start pose (max |Δq|={max_err:.4f} rad); '
                    'nothing to send'
                )
                self.exit_code = 0
                rclpy.shutdown()
                return

            traj = self._build_trajectory()
            self.publisher.publish(traj)
            self._published = True
            self._published_at = now
            self.get_logger().info(
                f'Published move_to_start: {len(traj.points)} pts, '
                f'T={self._duration_s:.2f}s, speed_factor={self.speed_factor}, '
                f'max|Δq|={max_err:.3f} rad'
            )
            return

        assert self._published_at is not None
        if self._saw_succeeded:
            self.get_logger().info(
                'PASS: JTC SUCCEEDED — moved to official start pose'
            )
            self.exit_code = 0
            rclpy.shutdown()
            return

        if now - self._published_at > self.wait_for_result_s:
            detail = (
                f'executing_seen={self._saw_executing} '
                f'succeeded_seen={self._saw_succeeded}'
            )
            self.get_logger().error(
                f'FAIL: no JTC SUCCEEDED within {self.wait_for_result_s:.0f}s '
                f'({detail})'
            )
            self.exit_code = 1
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = MoveToStart()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
