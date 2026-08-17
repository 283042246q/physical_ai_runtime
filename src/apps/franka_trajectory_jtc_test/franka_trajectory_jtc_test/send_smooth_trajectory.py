#!/usr/bin/env python3
"""Smooth trajectory example using the same API as diffusion_planner_example.

API (unchanged from policy_inference/examples/diffusion_planner_example.py):

  1. model          — SmoothTrajectoryPlanner  (like DiffusionPlanner)
  2. observe state  — on_joint_state → self.state
  3. compute        — planner.plan(request) → result dict
  4. to_msg         — result_to_ros(result) → JointTrajectory
  5. send           — publisher.publish(msg)

Exit policy matches ``move_to_start.py``: ``rclpy.spin`` until JTC SUCCEEDED
(or timeout), then ``rclpy.shutdown()``.

Request (dict):
  robot_model, planning_frame, joint_names
  q_pos_start / q_pos_goal     [7]
  q_vel_start / q_vel_goal     [7]
  q_acc_start / q_acc_goal     [7]

Result (dict):
  positions        [T, D]   rad
  velocities       [T, D]   rad/s
  accelerations    [T, D]   rad/s^2
  time_from_start  [T]      s, strictly increasing
  joint_names      [D]

Run after ``trajectory_executor.launch.py`` is up:

  ros2 run franka_trajectory_jtc_test send_smooth_trajectory.py
"""

from __future__ import annotations

import sys
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
import numpy as np
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


def _quintic(s: float) -> tuple[float, float, float]:
    """Quintic blend s in [0,1] → (scale, dscale/ds, d2scale/ds2)."""
    s = float(np.clip(s, 0.0, 1.0))
    pos = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    d1 = 30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4
    d2 = 60.0 * s - 180.0 * s**2 + 120.0 * s**3
    return pos, d1, d2


class SmoothTrajectoryPlanner:
    """Same role as DiffusionPlanner: plan(request) → result dict."""

    def __init__(
        self,
        joint_index: int = 3,
        amplitude_rad: float = 0.25,
        duration_s: float = 4.0,
        num_points: int = 80,
    ) -> None:
        self.joint_index = joint_index
        self.amplitude_rad = amplitude_rad
        self.duration_s = duration_s
        self.num_points = num_points

    def plan(self, request: dict) -> dict:
        """Run one start→goal plan. Same signature as DiffusionPlanner.plan."""
        start = np.asarray(request['q_pos_start'], dtype=np.float64)
        goal = np.asarray(request['q_pos_goal'], dtype=np.float64)
        delta = goal - start
        if np.allclose(delta, 0.0):
            delta = np.zeros_like(start)
            delta[self.joint_index] = self.amplitude_rad

        num_points = self.num_points
        duration_s = self.duration_s
        half = duration_s * 0.5
        times = np.linspace(0.0, duration_s, num_points, dtype=np.float64)
        positions = np.zeros((num_points, start.size), dtype=np.float64)
        velocities = np.zeros_like(positions)
        accelerations = np.zeros_like(positions)

        for i, t in enumerate(times):
            if t <= half:
                s = t / half if half > 0.0 else 1.0
                scale, d1, d2 = _quintic(s)
                ds_dt = d1 / half if half > 0.0 else 0.0
                d2s_dt2 = d2 / (half * half) if half > 0.0 else 0.0
            else:
                s = (t - half) / half if half > 0.0 else 1.0
                scale_fwd, d1, d2 = _quintic(s)
                scale = 1.0 - scale_fwd
                ds_dt = -d1 / half if half > 0.0 else 0.0
                d2s_dt2 = -d2 / (half * half) if half > 0.0 else 0.0

            positions[i] = start + delta * scale
            velocities[i] = delta * ds_dt
            accelerations[i] = delta * d2s_dt2

        positions[-1] = start
        velocities[-1] = 0.0
        accelerations[-1] = 0.0

        return {
            'positions': positions,                    # [T, D]
            'velocities': velocities,                  # [T, D]
            'accelerations': accelerations,            # [T, D]
            'time_from_start': times,                  # [T]
            'joint_names': list(request['joint_names']),
        }


class SmoothTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__('send_smooth_trajectory')
        self.declare_parameter('joint_names', FR3_JOINTS)
        self.declare_parameter('joint_state_topic', '/franka/joint_states')
        self.declare_parameter('robot_model', 'franka_fr3')
        self.declare_parameter('planning_frame', 'fr3_link0')
        self.declare_parameter(
            'goal_topic',
            '/action_sources/trajectory_test/joint_trajectory_goal',
        )
        self.declare_parameter(
            'jtc_status_topic',
            '/fr3_arm_controller/follow_joint_trajectory/_action/status',
        )
        self.declare_parameter('joint_index', 3)
        self.declare_parameter('amplitude_rad', 0.25)
        self.declare_parameter('duration_s', 4.0)
        self.declare_parameter('num_points', 80)
        self.declare_parameter('wait_for_em_s', 15.0)
        self.declare_parameter('wait_for_result_s', 20.0)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.wait_for_em_s = float(self.get_parameter('wait_for_em_s').value)
        self.wait_for_result_s = float(
            self.get_parameter('wait_for_result_s').value
        )

        # 1. model
        self.planner = SmoothTrajectoryPlanner(
            joint_index=int(self.get_parameter('joint_index').value),
            amplitude_rad=float(self.get_parameter('amplitude_rad').value),
            duration_s=float(self.get_parameter('duration_s').value),
            num_points=int(self.get_parameter('num_points').value),
        )
        self.state: np.ndarray | None = None
        self.published = False
        self._published_at: float | None = None
        self._saw_executing = False
        self._saw_succeeded = False
        self.exit_code = 1
        self._started = time.monotonic()

        goal_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter('goal_topic').value),
            goal_qos,
        )
        self._status_sub = self.create_subscription(
            GoalStatusArray,
            str(self.get_parameter('jtc_status_topic').value),
            self._on_status,
            10,
        )
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'Waiting for joint states, EM subscription, and JTC status…'
        )

    # 2. model obtains external state
    def on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.joint_names):
            return
        self.state = np.asarray(
            [positions[name] for name in self.joint_names], dtype=np.float64
        )

    def _on_status(self, msg: GoalStatusArray) -> None:
        statuses = {entry.status for entry in msg.status_list}
        self._saw_executing |= GoalStatus.STATUS_EXECUTING in statuses
        self._saw_succeeded |= GoalStatus.STATUS_SUCCEEDED in statuses

    def _log_trajectory(self, result: dict) -> None:
        positions = np.asarray(result['positions'], dtype=np.float64)
        velocities = np.asarray(result['velocities'], dtype=np.float64)
        accelerations = np.asarray(result['accelerations'], dtype=np.float64)
        times = np.asarray(result['time_from_start'], dtype=np.float64)
        joint_names = list(result['joint_names'])
        t_end = float(times[-1]) if times.size else 0.0

        self.get_logger().info(
            f'trajectory: T={positions.shape[0]} D={positions.shape[1]} '
            f'duration={t_end:.3f}s joints={joint_names}'
        )
        self.get_logger().info(
            f'  |q|max={np.max(np.abs(positions)):.4f} '
            f'|qd|max={np.max(np.abs(velocities)):.4f} '
            f'|qdd|max={np.max(np.abs(accelerations)):.4f}'
        )
        header = 'i  t[s]     ' + ' '.join(f'{n:>10s}' for n in joint_names)
        self.get_logger().info(f'  positions:\n{header}')
        for i, (t, q) in enumerate(zip(times, positions, strict=True)):
            q_str = ' '.join(f'{v:10.4f}' for v in q)
            self.get_logger().info(f'  {i:03d} {t:7.3f}  {q_str}')

        self.get_logger().info('  velocities:')
        for i, (t, qd) in enumerate(zip(times, velocities, strict=True)):
            qd_str = ' '.join(f'{v:10.4f}' for v in qd)
            self.get_logger().info(f'  {i:03d} {t:7.3f}  {qd_str}')

    def _tick(self) -> None:
        now = time.monotonic()
        if not self.published:
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
                and self.state is not None
            )
            if not ready:
                return

            zeros = np.zeros(len(self.joint_names), dtype=np.float64)
            q_goal = self.state.copy()
            q_goal[self.planner.joint_index] += self.planner.amplitude_rad

            request = {
                'robot_model': str(self.get_parameter('robot_model').value),
                'planning_frame': str(self.get_parameter('planning_frame').value),
                'joint_names': self.joint_names,
                'q_pos_start': self.state.copy(),
                'q_pos_goal': q_goal,
                'q_vel_start': zeros.copy(),
                'q_vel_goal': zeros.copy(),
                'q_acc_start': zeros.copy(),
                'q_acc_goal': zeros.copy(),
            }
            # 3. compute
            result = self.planner.plan(request)
            self._log_trajectory(result)
            # 4. to_msg + 5. send
            msg = self.result_to_ros(result)
            self.publisher.publish(msg)
            self.published = True
            self._published_at = now
            self.get_logger().info(
                f'Published smooth plan with {result["positions"].shape[0]} waypoints'
            )
            return

        assert self._published_at is not None
        if self._saw_succeeded:
            self.get_logger().info(
                'PASS: JTC reported SUCCEEDED after smooth trajectory goal'
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

    # 4. encapsulate as msg
    def result_to_ros(self, result: dict) -> JointTrajectory:
        positions = np.asarray(result['positions'], dtype=np.float64)
        velocities = np.asarray(result['velocities'], dtype=np.float64)
        accelerations = np.asarray(result['accelerations'], dtype=np.float64)
        times = np.asarray(result['time_from_start'], dtype=np.float64)
        joint_names = list(result['joint_names'])

        if joint_names != self.joint_names:
            raise ValueError('result joint_names must match the configured order')
        if positions.ndim != 2 or positions.shape[1] != len(self.joint_names):
            raise ValueError('positions must have shape [T,D]')
        if velocities.shape != positions.shape or accelerations.shape != positions.shape:
            raise ValueError('velocities/accelerations must match positions shape')
        if times.shape != (positions.shape[0],) or np.any(np.diff(times) <= 0.0):
            raise ValueError('time_from_start must be shape [T] and strictly increasing')
        if not all(
            np.isfinite(array).all()
            for array in (positions, velocities, accelerations, times)
        ):
            raise ValueError('trajectory contains NaN or Inf')

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = joint_names
        for position, velocity, acceleration, time_s in zip(
            positions, velocities, accelerations, times, strict=True
        ):
            point = JointTrajectoryPoint()
            point.positions = position.tolist()
            point.velocities = velocity.tolist()
            point.accelerations = acceleration.tolist()
            nanoseconds = round(float(time_s) * 1e9)
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = nanoseconds % 1_000_000_000
            msg.points.append(point)
        return msg


def main() -> None:
    rclpy.init()
    node = SmoothTrajectoryNode()
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
