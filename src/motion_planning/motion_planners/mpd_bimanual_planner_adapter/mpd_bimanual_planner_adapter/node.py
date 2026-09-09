"""Resident Marvin bimanual ROS 2 action adapter (no torch/CUDA imports)."""
from __future__ import annotations

import threading
import time

from .backend import BimanualPlannerBackend, WorkerRejected
from .contract import validate_goal
from .one_shot import build_request, canonical_joint_state
from .trajectory_adapter import trajectory_from_result


def main(args=None):
    try:
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from manipulation_planning_interfaces.action import PlanBimanualTrajectory
        from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from tf2_ros import Buffer, TransformListener
    except ImportError as error:
        raise RuntimeError("source ROS2 before starting the Marvin adapter") from error

    class BimanualPlannerNode(Node):
        def __init__(self):
            super().__init__("mpd_bimanual_planner_adapter")
            self.declare_parameter("worker_socket", "/tmp/mpd_marvin_bimanual_static.sock")
            self.declare_parameter("worker_timeout_s", 900.0)
            self.declare_parameter("execute", False)
            self.declare_parameter("joint_state_topic", "/joint_states")
            self.declare_parameter("max_joint_state_age_s", 0.5)
            self.declare_parameter("max_start_drift_rad", 0.01)
            self.declare_parameter(
                "trajectory_action", "/bimanual_arm_jtc/follow_joint_trajectory"
            )
            self.backend = BimanualPlannerBackend(
                str(self.get_parameter("worker_socket").value),
                timeout_s=float(self.get_parameter("worker_timeout_s").value),
            )
            self.execute_enabled = bool(self.get_parameter("execute").value)
            self.max_age_s = float(self.get_parameter("max_joint_state_age_s").value)
            self.max_drift = float(self.get_parameter("max_start_drift_rad").value)
            self._snapshot = None
            self._request_seq = 0
            self._planning_lock = threading.Lock()
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.trajectory_client = ActionClient(
                self,
                FollowJointTrajectory,
                str(self.get_parameter("trajectory_action").value),
            )
            self.create_subscription(
                JointState,
                str(self.get_parameter("joint_state_topic").value),
                self._joint_state,
                qos_profile_sensor_data,
            )
            self._server = ActionServer(
                self,
                PlanBimanualTrajectory,
                "plan",
                execute_callback=self._execute,
                goal_callback=self._goal,
                cancel_callback=self._cancel,
            )

        def _joint_state(self, message):
            try:
                source_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                self._snapshot = canonical_joint_state(
                    message.name,
                    message.position,
                    received_monotonic_ns=time.monotonic_ns(),
                    source_stamp_ns=source_ns,
                )
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=2.0)

        def _goal(self, goal):
            try:
                validate_goal(goal)
            except ValueError as error:
                self.get_logger().error(str(error))
                return GoalResponse.REJECT
            return GoalResponse.REJECT if self._planning_lock.locked() else GoalResponse.ACCEPT

        @staticmethod
        def _cancel(_goal_handle):
            return CancelResponse.ACCEPT

        @staticmethod
        def _pose_payload(message):
            pose = message.pose
            return {
                "frame_id": "world",
                "pose_xyzw": [
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                ],
            }

        def _transform_goal(self, message):
            if message.header.frame_id == "world":
                stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                return self._pose_payload(message), stamp
            transform = self.tf_buffer.lookup_transform(
                "world", message.header.frame_id, rclpy.time.Time()
            )
            from tf2_geometry_msgs import do_transform_pose_stamped
            transformed = do_transform_pose_stamped(message, transform)
            stamp = int(transform.header.stamp.sec) * 1_000_000_000 + int(transform.header.stamp.nanosec)
            return self._pose_payload(transformed), stamp

        def _request(self, goal, snapshot, deadline_ns):
            left = right = None
            stamps = [snapshot.source_stamp_ns]
            if goal.has_left_goal_pose:
                left, stamp = self._transform_goal(goal.left_goal_pose)
                stamps.append(stamp)
            if goal.has_right_goal_pose:
                right, stamp = self._transform_goal(goal.right_goal_pose)
                stamps.append(stamp)
            q_goal = list(goal.q_goal) if len(goal.q_goal) else None
            return build_request(
                request_id=goal.request_id,
                q_start=snapshot.positions,
                q_goal=q_goal,
                left_goal_pose=left,
                right_goal_pose=right,
                deadline_unix_ns=deadline_ns,
                seed=12345,
                tf_stamp_ns=min(stamps),
            )

        async def _execute_atomic(self, trajectory):
            if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError("bimanual_arm_jtc action unavailable")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            handle = await self.trajectory_client.send_goal_async(goal)
            if not handle.accepted:
                raise RuntimeError("atomic 14-joint trajectory goal rejected")
            outcome = await handle.get_result_async()
            if outcome.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(
                    f"bimanual controller failed atomically: {outcome.result.error_string}"
                )

        async def _execute(self, goal_handle):
            result = PlanBimanualTrajectory.Result()
            result.request_id = goal_handle.request.request_id
            if not self._planning_lock.acquire(blocking=False):
                result.error_code = "BUSY"
                result.message = "another bimanual plan owns the non-reentrant worker"
                goal_handle.abort()
                return result
            try:
                if self._snapshot is None:
                    raise RuntimeError("no complete 14-joint state received")
                snapshot = self._snapshot
                snapshot.require_fresh(
                    now_monotonic_ns=time.monotonic_ns(), max_age_s=self.max_age_s
                )
                deadline_ns = time.time_ns() + int(goal_handle.request.planning_budget_s * 1e9)
                request = self._request(goal_handle.request, snapshot, deadline_ns)
                self._request_seq += 1
                payload = self.backend.plan(
                    request,
                    request_seq=self._request_seq,
                    deadline_unix_ns=deadline_ns,
                )
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = "CANCELLED"
                    return result
                if time.time_ns() >= deadline_ns:
                    raise RuntimeError("planning result arrived after deadline")
                current = self._snapshot
                current.require_fresh(
                    now_monotonic_ns=time.monotonic_ns(), max_age_s=self.max_age_s
                )
                drift = max(abs(a - b) for a, b in zip(current.positions, snapshot.positions))
                if drift > self.max_drift:
                    raise RuntimeError(f"start drift {drift:.6g} invalidated result")
                trajectory = trajectory_from_result(
                    payload, q_start=snapshot.positions, max_start_error_rad=1e-5
                )
                result.trajectory = trajectory
                result.world_version = int(payload.get("world_version", 0))
                validation = payload.get("validation") or {}
                result.min_clearance_m = float(
                    min(
                        validation.get("minimum_environment_clearance_m", float("inf")),
                        validation.get("minimum_left_self_clearance_m", float("inf")),
                        validation.get("minimum_right_self_clearance_m", float("inf")),
                        validation.get("minimum_interarm_clearance_m", float("inf")),
                    )
                )
                if self.execute_enabled and goal_handle.request.execute:
                    await self._execute_atomic(trajectory)
                result.success = True
                result.error_code = ""
                result.message = "executed" if self.execute_enabled and goal_handle.request.execute else "plan_only"
                goal_handle.succeed()
                return result
            except WorkerRejected as error:
                result.error_code = error.status
                result.message = str(error)
                goal_handle.abort()
                return result
            except Exception as error:
                result.error_code = "FAILED_CLOSED"
                result.message = str(error)
                goal_handle.abort()
                return result
            finally:
                self._planning_lock.release()

    rclpy.init(args=args)
    node = BimanualPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
