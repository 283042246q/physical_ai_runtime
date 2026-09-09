"""Phase-3 Marvin one-shot ROS 2 planner and atomic 14-joint executor."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time
import uuid

from .contract import JOINT_NAMES
from .one_shot import OneShotMpdPlanner, build_request, canonical_joint_state


DEFAULT_Q_GOAL = [
    0.15, -0.10, 0.20, -0.15, 0.10, 0.05, -0.05,
    -0.15, 0.10, -0.20, 0.15, -0.10, -0.05, 0.05,
]
DEFAULT_INFER_ONCE = Path(
    "/home/eric/Projects/MotionPlanningDiffusion/mpd/scripts/runtime/"
    "infer_once_marvin_bimanual.py"
)


def main(args=None):
    try:
        import numpy as np
        import rclpy
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from geometry_msgs.msg import PoseStamped
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from tf2_ros import Buffer, TransformListener
    except ImportError as error:
        raise RuntimeError("source the ROS 2 workspace before starting one-shot") from error

    class MarvinOneShotNode(Node):
        def __init__(self):
            super().__init__("marvin_mpd_bimanual_one_shot")
            self.declare_parameter("plan_only", True)
            self.declare_parameter("joint_state_topic", "/joint_states")
            self.declare_parameter(
                "trajectory_action", "/bimanual_arm_jtc/follow_joint_trajectory"
            )
            self.declare_parameter("fault_topic", "/marvin/bimanual_fault")
            self.declare_parameter("planning_frame", "world")
            self.declare_parameter("goal_type", "joint")
            self.declare_parameter("q_goal", DEFAULT_Q_GOAL)
            self.declare_parameter("left_goal_frame", "world")
            self.declare_parameter("right_goal_frame", "world")
            self.declare_parameter("left_goal_pose", [0.50, 0.30, 0.60, 0.0, 0.0, 0.0, 1.0])
            self.declare_parameter("right_goal_pose", [0.50, -0.30, 0.60, 0.0, 0.0, 0.0, 1.0])
            self.declare_parameter("seed", 12345)
            self.declare_parameter("device", "cuda:0")
            self.declare_parameter("output_root", "/tmp/mpd-marvin-bimanual-one-shot")
            self.declare_parameter("mpd_conda", "/home/eric/anaconda3/bin/conda")
            self.declare_parameter("mpd_conda_env", "mpd-splines-public")
            self.declare_parameter("mpd_conda_prefix", "/home/eric/anaconda3/envs/mpd-splines-public")
            self.declare_parameter("mpd_infer_once", str(DEFAULT_INFER_ONCE))
            self.declare_parameter("planning_timeout_s", 900.0)
            self.declare_parameter("max_joint_state_age_s", 0.5)
            self.declare_parameter("max_start_drift_rad", 0.01)

            if str(self.get_parameter("planning_frame").value) != "world":
                raise ValueError("Marvin Warehouse planning_frame must be world")
            self.plan_only = bool(self.get_parameter("plan_only").value)
            self.max_age_s = float(self.get_parameter("max_joint_state_age_s").value)
            self.max_drift = float(self.get_parameter("max_start_drift_rad").value)
            self.timeout_s = float(self.get_parameter("planning_timeout_s").value)
            self.planner = OneShotMpdPlanner(
                conda=Path(str(self.get_parameter("mpd_conda").value)),
                conda_env=str(self.get_parameter("mpd_conda_env").value),
                conda_prefix=Path(str(self.get_parameter("mpd_conda_prefix").value)),
                infer_once=Path(str(self.get_parameter("mpd_infer_once").value)),
                output_root=Path(str(self.get_parameter("output_root").value)),
                device=str(self.get_parameter("device").value),
                timeout_s=self.timeout_s,
            )
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.action_client = ActionClient(
                self,
                FollowJointTrajectory,
                str(self.get_parameter("trajectory_action").value),
            )
            self.create_subscription(
                JointState,
                str(self.get_parameter("joint_state_topic").value),
                self._on_joint_state,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("fault_topic").value),
                self._on_fault,
                10,
            )
            self._snapshot = None
            self._planned_from = None
            self._future = None
            self._goal_handle = None
            self._started = False
            self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="marvin-mpd")
            self.create_timer(0.05, self._tick)
            self.get_logger().info(f"waiting for 14-joint state; plan_only={self.plan_only}")

        def _on_joint_state(self, message):
            try:
                stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                self._snapshot = canonical_joint_state(
                    message.name,
                    message.position,
                    received_monotonic_ns=time.monotonic_ns(),
                    source_stamp_ns=stamp_ns,
                )
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=2.0)

        def _on_fault(self, message):
            if not bool(message.data) or self._goal_handle is None:
                return
            self.get_logger().error("arm fault: cancelling the whole 14-joint goal")
            self._goal_handle.cancel_goal_async()

        @staticmethod
        def _pose_payload(pose):
            return {
                "frame_id": "world",
                "pose_xyzw": [
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z,
                    pose.orientation.w,
                ],
            }

        def _world_goal(self, side):
            source_frame = str(self.get_parameter(f"{side}_goal_frame").value)
            values = list(self.get_parameter(f"{side}_goal_pose").value)
            if len(values) != 7 or not np.isfinite(values).all():
                raise ValueError(f"{side}_goal_pose must contain seven finite values")
            goal = PoseStamped()
            goal.header.frame_id = source_frame
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = values[:3]
            goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = values[3:]
            if source_frame == "world":
                return self._pose_payload(goal.pose), self.get_clock().now().nanoseconds
            transform = self.tf_buffer.lookup_transform("world", source_frame, rclpy.time.Time())
            from tf2_geometry_msgs import do_transform_pose
            transformed = do_transform_pose(goal.pose, transform)
            stamp_ns = int(transform.header.stamp.sec) * 1_000_000_000 + int(transform.header.stamp.nanosec)
            return self._pose_payload(transformed), stamp_ns

        def _make_request(self):
            self._snapshot.require_fresh(
                now_monotonic_ns=time.monotonic_ns(), max_age_s=self.max_age_s
            )
            goal_type = str(self.get_parameter("goal_type").value)
            if goal_type == "joint":
                q_goal = list(self.get_parameter("q_goal").value)
                if len(q_goal) != 14 or not np.isfinite(q_goal).all():
                    raise ValueError("q_goal must contain fourteen finite values")
                left = right = None
                tf_stamp_ns = self._snapshot.source_stamp_ns
            elif goal_type == "cartesian":
                q_goal = None
                left, left_stamp = self._world_goal("left")
                right, right_stamp = self._world_goal("right")
                tf_stamp_ns = min(left_stamp, right_stamp)
            else:
                raise ValueError("goal_type must be joint or cartesian")
            return build_request(
                request_id=str(uuid.uuid4()),
                q_start=self._snapshot.positions,
                q_goal=q_goal,
                left_goal_pose=left,
                right_goal_pose=right,
                deadline_unix_ns=time.time_ns() + int(self.timeout_s * 1e9),
                seed=int(self.get_parameter("seed").value),
                tf_stamp_ns=tf_stamp_ns,
            )

        @staticmethod
        def _duration(seconds):
            message = Duration()
            message.sec = int(seconds)
            message.nanosec = int(round((seconds - int(seconds)) * 1e9))
            return message

        def _trajectory(self, result):
            trajectory = JointTrajectory()
            trajectory.joint_names = list(JOINT_NAMES)
            for q, dq, ddq, stamp in zip(
                result["positions"], result["velocities"],
                result["accelerations"], result["time_from_start"]
            ):
                point = JointTrajectoryPoint()
                point.positions = q
                point.velocities = dq
                point.accelerations = ddq
                point.time_from_start = self._duration(float(stamp))
                trajectory.points.append(point)
            return trajectory

        def _tick(self):
            if not self._started and self._snapshot is not None:
                try:
                    request = self._make_request()
                    self._planned_from = np.asarray(request["q_start"], dtype=float)
                    self._future = self._pool.submit(self.planner.plan, request)
                    self._started = True
                    self.get_logger().info(f"planning request_id={request['request_id']}")
                except Exception as error:
                    self.get_logger().error(str(error))
                return
            if self._future is None or not self._future.done():
                return
            future, self._future = self._future, None
            try:
                result, output_dir = future.result()
                self._snapshot.require_fresh(
                    now_monotonic_ns=time.monotonic_ns(), max_age_s=self.max_age_s
                )
                drift = float(np.max(np.abs(np.asarray(self._snapshot.positions) - self._planned_from)))
                if drift > self.max_drift:
                    raise RuntimeError(f"start drift {drift:.6g} rad invalidated plan")
                if self.plan_only:
                    self.get_logger().info(f"validated plan-only artifact: {output_dir}")
                    return
                if not self.action_client.wait_for_server(timeout_sec=10.0):
                    raise RuntimeError("14-joint trajectory action is unavailable")
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = self._trajectory(result)
                send = self.action_client.send_goal_async(goal)
                send.add_done_callback(self._goal_response)
            except Exception as error:
                self.get_logger().error(f"one-shot failed closed: {error}")

        def _goal_response(self, future):
            self._goal_handle = future.result()
            if not self._goal_handle.accepted:
                self.get_logger().error("whole bimanual trajectory was rejected")
                return
            self.get_logger().info("one atomic 14-joint trajectory goal accepted")

    rclpy.init(args=args)
    node = MarvinOneShotNode()
    try:
        rclpy.spin(node)
    finally:
        node._pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()
