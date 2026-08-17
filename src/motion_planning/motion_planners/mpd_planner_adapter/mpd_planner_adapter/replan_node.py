"""ROS 2 node for bounded, asynchronous resident-MPD replanning."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from collections import deque
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt64
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from manipulation_motion_planning.contracts import JointTarget, PoseTarget, StartState

from .backend import EXPECTED_JOINT_NAMES, MpdGlobalTrajectoryBackend
from .coordinator import LatestOnlyPlanner
from .execution import JtcHandoffManager
from .handoff import HandoffValidationError, splice_for_handoff
from .trajectory import TimedPlan


@dataclass(frozen=True)
class PlanningJob:
    generation: int
    world_version: int
    start: StartState
    target: PoseTarget | JointTarget
    handoff_unix_ns: int
    deadline_unix_ns: int


def _stamp_s(message_stamp: Any) -> float:
    return float(message_stamp.sec) + float(message_stamp.nanosec) * 1e-9


def _duration(seconds: float):
    from builtin_interfaces.msg import Duration

    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


def _time_from_unix_ns(unix_ns: int):
    from builtin_interfaces.msg import Time

    return Time(sec=unix_ns // 1_000_000_000, nanosec=unix_ns % 1_000_000_000)


class MpdReplanNode(Node):
    def __init__(self) -> None:
        super().__init__("mpd_replanner")
        self.declare_parameter("socket_path", "/tmp/mpd-runtime.sock")
        self.declare_parameter("scene_id", "EnvWarehouseExtraObjectsV00")
        self.declare_parameter("seed", 0)
        self.declare_parameter("worker_timeout_s", 1.5)
        self.declare_parameter("plan_rate_hz", 1.0)
        self.declare_parameter("planning_budget_s", 0.8)
        self.declare_parameter("commit_margin_s", 0.1)
        self.declare_parameter("max_state_age_s", 0.25)
        self.declare_parameter("plan_only", True)
        self.declare_parameter("joint_state_topic", "/franka/joint_states")
        self.declare_parameter("planned_trajectory_topic", "~/planned_trajectory")
        self.declare_parameter("target_pose_xyzw", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter(
            "jtc_action_name", "/franka_arm_jtc/follow_joint_trajectory"
        )
        self.declare_parameter("command_lead_s", 0.05)
        self.declare_parameter("prefix_dt_s", 0.05)
        self.declare_parameter("max_start_drift_rad", 0.10)
        self.declare_parameter("max_handoff_speed_rad_s", 0.20)
        self.declare_parameter("max_q_jump_rad", 0.03)
        self.declare_parameter("max_dq_jump_rad_s", 0.20)
        self.declare_parameter("max_ddq_jump_rad_s2", 2.0)

        self._planning_budget_s = float(self.get_parameter("planning_budget_s").value)
        self._commit_margin_s = float(self.get_parameter("commit_margin_s").value)
        self._max_state_age_s = float(self.get_parameter("max_state_age_s").value)
        plan_rate_hz = float(self.get_parameter("plan_rate_hz").value)
        if plan_rate_hz <= 0.0:
            raise ValueError("plan_rate_hz must be positive")
        if self._planning_budget_s <= self._commit_margin_s:
            raise ValueError("planning_budget_s must exceed commit_margin_s")
        self._plan_only = bool(self.get_parameter("plan_only").value)
        self._command_lead_s = float(self.get_parameter("command_lead_s").value)
        self._splice_options = {
            "prefix_dt_s": float(self.get_parameter("prefix_dt_s").value),
            "max_start_drift_rad": float(
                self.get_parameter("max_start_drift_rad").value
            ),
            "max_handoff_speed_rad_s": float(
                self.get_parameter("max_handoff_speed_rad_s").value
            ),
            "max_q_jump_rad": float(self.get_parameter("max_q_jump_rad").value),
            "max_dq_jump_rad_s": float(
                self.get_parameter("max_dq_jump_rad_s").value
            ),
            "max_ddq_jump_rad_s2": float(
                self.get_parameter("max_ddq_jump_rad_s2").value
            ),
        }

        self._backend = MpdGlobalTrajectoryBackend(
            str(self.get_parameter("socket_path").value),
            scene_id=str(self.get_parameter("scene_id").value),
            seed=int(self.get_parameter("seed").value),
            timeout_s=float(self.get_parameter("worker_timeout_s").value),
        )
        self._backend_ready = False
        self._planner = LatestOnlyPlanner(self._plan_job)
        # The worker can outlive this ROS node.  A Unix-nanosecond epoch keeps
        # request_seq newer after a node restart instead of replaying 1, 2, ...
        self._generation = time.time_ns()
        self._world_version = 0
        self._state: StartState | None = None
        self._state_received_monotonic = 0.0
        self._target: PoseTarget | JointTarget | None = None
        self._active_plan: TimedPlan | None = None
        self._candidate_plans: dict[int, TimedPlan] = {}
        self._counters = {
            "submitted": 0,
            "accepted": 0,
            "invalid": 0,
            "superseded": 0,
            "deadline_miss": 0,
            "worker_error": 0,
            "handoff_rejected": 0,
            "goal_submitted": 0,
            "goal_accepted": 0,
            "goal_terminal": 0,
        }
        self._latencies_s: deque[float] = deque(maxlen=512)

        try:
            target_pose = list(self.get_parameter("target_pose_xyzw").value)
        except ParameterUninitializedException:
            # The production launch intentionally starts without a target.
            target_pose = []
        if target_pose:
            if len(target_pose) != 7 or not all(math.isfinite(x) for x in target_pose):
                raise ValueError("target_pose_xyzw must be empty or [x,y,z,qx,qy,qz,qw]")
            x, y, z, qx, qy, qz, qw = target_pose
            norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if norm < 1e-9:
                raise ValueError("target_pose_xyzw quaternion has zero norm")
            self._target = PoseTarget(
                (x, y, z), (qw / norm, qx / norm, qy / norm, qz / norm)
            )

        self._trajectory_publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("planned_trajectory_topic").value),
            1,
        )
        self._diagnostics_publisher = self.create_publisher(String, "~/diagnostics", 10)
        self._execution = (
            None
            if self._plan_only
            else JtcHandoffManager(
                self,
                str(self.get_parameter("jtc_action_name").value),
                on_accepted=self._on_goal_accepted,
                on_terminal=self._on_goal_terminal,
            )
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(PoseStamped, "~/pose_target", self._on_pose_target, 1)
        self.create_subscription(JointState, "~/joint_target", self._on_joint_target, 1)
        self.create_subscription(Bool, "~/stop", self._on_stop, 1)
        stop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, "/mpd/emergency_stop", self._on_stop, stop_qos
        )
        self.create_subscription(UInt64, "~/world_version", self._on_world_version, 1)
        self.create_service(Trigger, "~/safe_stop", self._on_safe_stop)
        self.create_timer(1.0 / plan_rate_hz, self._schedule)
        self.create_timer(0.02, self._drain)
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            "resident MPD replanner started "
            f"(plan_only={self._plan_only}, rate={plan_rate_hz:.2f} Hz)"
        )

    def _on_joint_state(self, message: JointState) -> None:
        try:
            positions = dict(zip(message.name, message.position))
            velocities = dict(zip(message.name, message.velocity))
            q = [float(positions[name]) for name in EXPECTED_JOINT_NAMES]
            dq = (
                [float(velocities[name]) for name in EXPECTED_JOINT_NAMES]
                if all(name in velocities for name in EXPECTED_JOINT_NAMES)
                else [0.0] * 7
            )
            if not all(math.isfinite(value) for value in q + dq):
                raise ValueError("non-finite state")
        except (KeyError, ValueError) as error:
            self.get_logger().warning(f"ignored incomplete joint state: {error}")
            return
        stamp = _stamp_s(message.header.stamp)
        self._state = StartState(list(EXPECTED_JOINT_NAMES), q, dq, stamp)
        self._state_received_monotonic = time.monotonic()

    def _on_pose_target(self, message: PoseStamped) -> None:
        p, q = message.pose.position, message.pose.orientation
        norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
        if not math.isfinite(norm) or norm < 1e-9:
            self.get_logger().warning("ignored pose target with invalid quaternion")
            return
        self._target = PoseTarget(
            (p.x, p.y, p.z),
            (q.w / norm, q.x / norm, q.y / norm, q.z / norm),
            _stamp_s(message.header.stamp),
        )
        self._invalidate_current_work()

    def _on_joint_target(self, message: JointState) -> None:
        try:
            by_name = dict(zip(message.name, message.position))
            target = [float(by_name[name]) for name in EXPECTED_JOINT_NAMES]
            if not all(math.isfinite(value) for value in target):
                raise ValueError("non-finite target")
        except (KeyError, ValueError) as error:
            self.get_logger().warning(f"ignored invalid joint target: {error}")
            return
        self._target = JointTarget(
            list(EXPECTED_JOINT_NAMES), target, _stamp_s(message.header.stamp)
        )
        self._invalidate_current_work()

    def _on_stop(self, message: Bool) -> None:
        if message.data:
            self._stop_planning()

    def _stop_planning(self) -> bool:
        self._target = None
        self._active_plan = None
        self._candidate_plans.clear()
        self._invalidate_current_work()
        canceled = self._execution.cancel() if self._execution is not None else False
        self.get_logger().warning("planning stopped; active/pending generations invalidated")
        return canceled

    def _on_safe_stop(self, request, response):
        canceled = self._stop_planning()
        response.success = True
        response.message = (
            "active JTC cancel requested"
            if canceled
            else "planner stopped; no active JTC goal"
        )
        return response

    def _on_world_version(self, message: UInt64) -> None:
        version = int(message.data)
        if version <= self._world_version:
            return
        self._world_version = version
        self._invalidate_current_work()

    def _invalidate_current_work(self) -> None:
        self._generation += 1
        self._planner.invalidate(self._generation)

    def _plan_job(self, job: PlanningJob):
        if not self._backend_ready:
            self._backend.warmup()
            self._backend_ready = True
        return self._backend.plan(
            job.start,
            job.target,
            {
                "request_seq": job.generation,
                "request_id": f"ros-generation-{job.generation}",
                "world_version": job.world_version,
                "handoff_unix_ns": job.handoff_unix_ns,
                "deadline_unix_ns": job.deadline_unix_ns,
            },
        )

    def _schedule(self) -> None:
        if self._state is None or self._target is None:
            return
        if time.monotonic() - self._state_received_monotonic > self._max_state_age_s:
            return
        now_ns = time.time_ns()
        handoff_ns = now_ns + int(self._planning_budget_s * 1e9)
        deadline_ns = handoff_ns - int(self._commit_margin_s * 1e9)
        try:
            start = (
                self._active_plan.predict(handoff_ns * 1e-9)
                if self._active_plan is not None
                else StartState(
                    self._state.joint_names,
                    self._state.positions,
                    self._state.velocities,
                    handoff_ns * 1e-9,
                )
            )
        except ValueError as error:
            self.get_logger().error(f"future-handoff prediction failed: {error}")
            return
        self._generation += 1
        job = PlanningJob(
            generation=self._generation,
            world_version=self._world_version,
            start=start,
            target=self._target,
            handoff_unix_ns=handoff_ns,
            deadline_unix_ns=deadline_ns,
        )
        self._planner.submit(job.generation, job)
        self._counters["submitted"] += 1

    def _drain(self) -> None:
        for completion in self._planner.drain():
            if completion.superseded or completion.generation != self._generation:
                self._counters["superseded"] += 1
                continue
            if completion.error is not None:
                self._counters["worker_error"] += 1
                self.get_logger().error(
                    f"generation {completion.generation} failed: {completion.error}"
                )
                continue
            self._latencies_s.append(completion.elapsed_s)
            result = completion.result
            if result is None or not result.valid:
                self._counters["invalid"] += 1
                reason = "empty result" if result is None else result.reason
                if reason and "deadline" in reason:
                    self._counters["deadline_miss"] += 1
                self.get_logger().warning(
                    f"generation {completion.generation} rejected: {reason}"
                )
                continue
            deadline_ns = completion.job.deadline_unix_ns
            if time.time_ns() >= deadline_ns:
                self._counters["deadline_miss"] += 1
                continue
            if self._plan_only:
                self._active_plan = TimedPlan(
                    result, completion.job.handoff_unix_ns * 1e-9
                )
                self._trajectory_publisher.publish(
                    self._to_message(result, completion.job.handoff_unix_ns)
                )
            elif not self._commit_execution(completion.job, result):
                continue
            self._counters["accepted"] += 1

    def _commit_execution(self, job: PlanningJob, result) -> bool:
        if self._execution is None or self._state is None:
            return False
        commit_start_s = time.time() + self._command_lead_s
        try:
            merged = splice_for_handoff(
                current_state=self._state,
                active_plan=self._active_plan,
                new_plan=result,
                commit_start_unix_s=commit_start_s,
                handoff_unix_s=job.handoff_unix_ns * 1e-9,
                **self._splice_options,
            )
        except HandoffValidationError as error:
            self._counters["handoff_rejected"] += 1
            self.get_logger().warning(f"plan {job.generation} handoff rejected: {error}")
            return False
        message = self._to_message(merged, int(commit_start_s * 1e9))
        self._trajectory_publisher.publish(message)
        if not self._execution.submit(job.generation, message):
            self._counters["worker_error"] += 1
            self.get_logger().error("JTC action server unavailable; goal not submitted")
            return False
        self._candidate_plans[job.generation] = TimedPlan(merged, commit_start_s)
        while len(self._candidate_plans) > 2:
            self._candidate_plans.pop(next(iter(self._candidate_plans)))
        self._counters["goal_submitted"] += 1
        return True

    def _on_goal_accepted(self, plan_id: int) -> None:
        candidate = self._candidate_plans.get(plan_id)
        if candidate is None:
            return
        self._active_plan = candidate
        self._counters["goal_accepted"] += 1

    def _on_goal_terminal(self, plan_id: int, state: str) -> None:
        self._candidate_plans.pop(plan_id, None)
        self._counters["goal_terminal"] += 1
        if self._execution is not None and self._execution.plan_id is None:
            self._active_plan = None
        if state in ("REJECTED", "ABORTED", "SEND_ERROR", "RESULT_ERROR"):
            self.get_logger().error(f"JTC plan {plan_id} entered terminal state {state}")

    @staticmethod
    def _to_message(result, handoff_unix_ns: int) -> JointTrajectory:
        message = JointTrajectory()
        message.header.stamp = _time_from_unix_ns(handoff_unix_ns)
        message.joint_names = list(result.joint_names or [])
        for point in result.points:
            ros_point = JointTrajectoryPoint()
            ros_point.positions = list(point.positions)
            if point.velocities is not None:
                ros_point.velocities = list(point.velocities)
            ros_point.time_from_start = _duration(point.time_from_start_s)
            message.points.append(ros_point)
        return message

    def _publish_diagnostics(self) -> None:
        ordered_latency = sorted(self._latencies_s)

        def percentile(fraction: float) -> float | None:
            if not ordered_latency:
                return None
            index = fraction * (len(ordered_latency) - 1)
            lower = int(math.floor(index))
            upper = int(math.ceil(index))
            alpha = index - lower
            return ordered_latency[lower] * (1.0 - alpha) + ordered_latency[upper] * alpha

        message = String()
        message.data = json.dumps(
            {
                "state": "READY" if self._backend_ready else "WAITING_FOR_WORKER",
                "generation": self._generation,
                "world_version": self._world_version,
                "planner_active": self._planner.active,
                "pending_count": self._planner.pending_count,
                "has_state": self._state is not None,
                "has_target": self._target is not None,
                "plan_only": self._plan_only,
                "execution_state": (
                    "PLAN_ONLY" if self._execution is None else self._execution.state
                ),
                "active_plan_id": (
                    None if self._execution is None else self._execution.plan_id
                ),
                "pending_plan_id": (
                    None
                    if self._execution is None
                    else self._execution.pending_plan_id
                ),
                "last_goal_result_code": (
                    None
                    if self._execution is None
                    else self._execution.last_result_error_code
                ),
                "last_goal_result_text": (
                    None
                    if self._execution is None
                    else self._execution.last_result_error_string
                ),
                "latency_samples": len(ordered_latency),
                "latency_p50_s": percentile(0.50),
                "latency_p95_s": percentile(0.95),
                "latency_p99_s": percentile(0.99),
                **self._counters,
            },
            sort_keys=True,
        )
        self._diagnostics_publisher.publish(message)

    def destroy_node(self) -> bool:
        self._planner.close()
        if self._execution is not None:
            self._execution.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MpdReplanNode()
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
