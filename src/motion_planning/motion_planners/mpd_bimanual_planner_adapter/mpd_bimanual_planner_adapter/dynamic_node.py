"""Phase-5 latest-only snapshot replanning for the complete Marvin 14D goal."""
from __future__ import annotations

import asyncio
from pathlib import Path
import threading
import time

from .backend import BimanualPlannerBackend
from .collision_guard import CollisionGuard
from .contract import JOINT_NAMES, validate_goal
from .latest_world_buffer import LatestWorldBuffer
from .one_shot import build_request, canonical_joint_state
from .replan_coordinator import LatestOnlyReplanCoordinator
from .replay_recorder import DynamicReplayRecorder
from .trajectory_adapter import trajectory_from_result


def main(args=None):
    try:
        import rclpy
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from manipulation_planning_interfaces.action import PlanBimanualTrajectory
        from manipulation_planning_interfaces.msg import DynamicWorld
        from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    except ImportError as error:
        raise RuntimeError("source ROS2 before starting Marvin dynamic replanning") from error

    class DynamicNode(Node):
        def __init__(self):
            super().__init__("mpd_bimanual_dynamic_planner")
            self.declare_parameter("worker_socket", "/tmp/mpd_marvin_bimanual_dynamic.sock")
            self.declare_parameter("execute", False)
            self.declare_parameter("joint_state_topic", "/joint_states")
            self.declare_parameter("world_topic", "/marvin/dynamic_world")
            self.declare_parameter("clearance_topic", "/marvin/prefix_min_clearance")
            self.declare_parameter("fault_topic", "/marvin/bimanual_fault")
            self.declare_parameter("trajectory_action", "/bimanual_arm_jtc/follow_joint_trajectory")
            self.declare_parameter("max_joint_state_age_s", 0.5)
            self.declare_parameter("warning_clearance_m", 0.05)
            self.declare_parameter("brake_clearance_m", 0.02)
            self.declare_parameter("hold_duration_s", 0.25)
            self.declare_parameter("max_replans", 100)
            self.declare_parameter("record_root", "/tmp/mpd-marvin-bimanual-dynamic")
            self.backend = BimanualPlannerBackend(
                str(self.get_parameter("worker_socket").value), timeout_s=900.0
            )
            self.execute_enabled = bool(self.get_parameter("execute").value)
            self.max_age_s = float(self.get_parameter("max_joint_state_age_s").value)
            self.hold_duration_s = float(self.get_parameter("hold_duration_s").value)
            self.max_replans = int(self.get_parameter("max_replans").value)
            self.record_root = Path(str(self.get_parameter("record_root").value))
            self.world = LatestWorldBuffer(planning_frame="world", max_objects=16)
            self.coordinator = LatestOnlyReplanCoordinator(lambda request: request)
            self.guard = CollisionGuard(
                warning_clearance_m=float(self.get_parameter("warning_clearance_m").value),
                brake_clearance_m=float(self.get_parameter("brake_clearance_m").value),
            )
            self._joint = None
            self._request_seq = 0
            self._worker_world_version = 0
            self._brake_requested = False
            self._recorder = None
            self._action_lock = threading.Lock()
            group = ReentrantCallbackGroup()
            self.trajectory_client = ActionClient(
                self,
                FollowJointTrajectory,
                str(self.get_parameter("trajectory_action").value),
                callback_group=group,
            )
            self.create_subscription(
                JointState,
                str(self.get_parameter("joint_state_topic").value),
                self._joint_state,
                qos_profile_sensor_data,
                callback_group=group,
            )
            self.create_subscription(
                DynamicWorld,
                str(self.get_parameter("world_topic").value),
                self._world,
                qos_profile_sensor_data,
                callback_group=group,
            )
            self.create_subscription(
                Float64,
                str(self.get_parameter("clearance_topic").value),
                self._clearance,
                10,
                callback_group=group,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("fault_topic").value),
                self._fault,
                10,
                callback_group=group,
            )
            self.server = ActionServer(
                self,
                PlanBimanualTrajectory,
                "plan_dynamic",
                execute_callback=self._execute,
                goal_callback=self._goal,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=group,
            )

        def _joint_state(self, message):
            try:
                stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                self._joint = canonical_joint_state(
                    message.name, message.position,
                    received_monotonic_ns=time.monotonic_ns(), source_stamp_ns=stamp,
                )
                if self._recorder is not None:
                    self._recorder.record(
                        "joint_state",
                        {"joint_names": list(JOINT_NAMES), "positions": list(self._joint.positions)},
                    )
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=2.0)

        def _world(self, message):
            try:
                snapshot = self.world.update(message)
                self.coordinator.invalidate()
                if self._recorder is not None:
                    self._recorder.record("world", snapshot.to_worker_payload())
            except ValueError as error:
                self.get_logger().error(f"rejected DynamicWorld: {error}")

        def _clearance(self, message):
            action = self.guard.classify(min_clearance_m=float(message.data))
            if action != CollisionGuard.SAFE:
                self._brake_requested = action == CollisionGuard.BRAKE
                self.coordinator.invalidate()
                if self._recorder is not None:
                    self._recorder.record("prefix_guard", {"action": action, "clearance_m": float(message.data)})

        def _fault(self, message):
            if message.data:
                self._brake_requested = True
                self.coordinator.invalidate()
                if self._recorder is not None:
                    self._recorder.record("fault", {"whole_goal_cancelled": True})

        def _goal(self, goal):
            try:
                validate_goal(goal)
                return (
                    GoalResponse.REJECT
                    if self._action_lock.locked()
                    else GoalResponse.ACCEPT
                )
            except ValueError:
                return GoalResponse.REJECT

        @staticmethod
        def _pose(message):
            if message.header.frame_id != "world":
                raise ValueError("dynamic EE goals must already be transformed to world")
            pose = message.pose
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            return {
                "frame_id": "world",
                "pose_xyzw": [
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z,
                    pose.orientation.w,
                ],
            }, stamp

        def _request(self, goal, joint, snapshot, deadline):
            left = right = None
            stamps = [joint.source_stamp_ns, snapshot.stamp_unix_ns]
            if goal.has_left_goal_pose:
                left, stamp = self._pose(goal.left_goal_pose)
                stamps.append(stamp)
            if goal.has_right_goal_pose:
                right, stamp = self._pose(goal.right_goal_pose)
                stamps.append(stamp)
            request = build_request(
                request_id=goal.request_id,
                q_start=joint.positions,
                q_goal=list(goal.q_goal) if len(goal.q_goal) else None,
                left_goal_pose=left,
                right_goal_pose=right,
                deadline_unix_ns=deadline,
                seed=12345,
                tf_stamp_ns=min(stamps),
            )
            request["world_version"] = snapshot.world_version
            request["scene"] = {
                "source": "ros2_latest_only",
                "tf_stamp_ns": min(stamps),
            }
            return request

        async def _hold(self):
            if self._joint is None or not self.trajectory_client.server_is_ready():
                return
            trajectory = JointTrajectory(joint_names=list(JOINT_NAMES))
            for seconds in (0.0, self.hold_duration_s):
                point = JointTrajectoryPoint(positions=list(self._joint.positions))
                point.velocities = [0.0] * 14
                point.accelerations = [0.0] * 14
                point.time_from_start = Duration(
                    sec=int(seconds),
                    nanosec=int(round((seconds - int(seconds)) * 1e9)),
                )
                trajectory.points.append(point)
            goal = FollowJointTrajectory.Goal(trajectory=trajectory)
            handle = await self.trajectory_client.send_goal_async(goal)
            if handle.accepted:
                await handle.get_result_async()

        async def _execute_trajectory(self, trajectory, ticket, goal_handle):
            if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError("bimanual trajectory controller unavailable")
            command = FollowJointTrajectory.Goal(trajectory=trajectory)
            handle = await self.trajectory_client.send_goal_async(command)
            if not handle.accepted:
                raise RuntimeError("atomic 14D execution rejected")
            outcome = handle.get_result_async()
            while not outcome.done():
                latest = self.world.snapshot
                if goal_handle.is_cancel_requested or latest is None or not self.coordinator.accepts(
                    ticket, latest_world_version=latest.world_version
                ):
                    await handle.cancel_goal_async()
                    await self._hold()
                    if self._recorder is not None:
                        self._recorder.record(
                            "handoff",
                            {"mode": "cancel_hold_replan", "generation": ticket.generation},
                        )
                    return False
                await asyncio.sleep(0.02)
            result = outcome.result().result
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(f"whole 14D execution failed: {result.error_string}")
            return True

        async def _execute(self, goal_handle):
            Result = PlanBimanualTrajectory.Result
            response = Result(request_id=goal_handle.request.request_id)
            if not self._action_lock.acquire(blocking=False):
                response.error_code = "BUSY"
                response.message = "another latest-only bimanual goal is active"
                goal_handle.abort()
                return response
            self._recorder = DynamicReplayRecorder(
                self.record_root / f"{goal_handle.request.request_id}.json",
                request_id=goal_handle.request.request_id,
            )
            try:
                for _ in range(self.max_replans):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        response.error_code = "CANCELLED"
                        return response
                    snapshot = self.world.snapshot
                    joint = self._joint
                    if snapshot is None or joint is None:
                        await asyncio.sleep(0.02)
                        continue
                    joint.require_fresh(now_monotonic_ns=time.monotonic_ns(), max_age_s=self.max_age_s)
                    if time.time_ns() >= snapshot.valid_until_ns:
                        await asyncio.sleep(0.02)
                        continue
                    if snapshot.world_version != self._worker_world_version:
                        self._worker_world_version = self.backend.update_world(snapshot.to_worker_payload())
                    self._request_seq += 1
                    deadline = time.time_ns() + int(goal_handle.request.planning_budget_s * 1e9)
                    ticket = self.coordinator.begin(
                        request_seq=self._request_seq,
                        world_version=snapshot.world_version,
                        deadline_unix_ns=deadline,
                    )
                    request = self._request(goal_handle.request, joint, snapshot, deadline)
                    self._recorder.record("plan_start", {"generation": ticket.generation, "world_version": ticket.world_version})
                    payload = self.backend.plan(
                        request, request_seq=ticket.request_seq, deadline_unix_ns=deadline
                    )
                    latest = self.world.snapshot
                    if latest is None or not self.coordinator.accepts(
                        ticket, latest_world_version=latest.world_version
                    ):
                        self._recorder.record("plan_discard", {"generation": ticket.generation, "reason": "stale_latest_only"})
                        continue
                    trajectory = trajectory_from_result(payload, q_start=joint.positions)
                    response.trajectory = trajectory
                    response.world_version = ticket.world_version
                    if not self.execute_enabled or not goal_handle.request.execute:
                        response.success = True
                        response.message = "latest snapshot plan_only"
                        goal_handle.succeed()
                        return response
                    if await self._execute_trajectory(trajectory, ticket, goal_handle):
                        response.success = True
                        response.message = "latest snapshot execution completed"
                        goal_handle.succeed()
                        return response
                raise RuntimeError("maximum latest-only replans exceeded")
            except Exception as error:
                response.error_code = "FAILED_CLOSED"
                response.message = str(error)
                goal_handle.abort()
                return response
            finally:
                self._recorder = None
                self._action_lock.release()

    rclpy.init(args=args)
    node = DynamicNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
