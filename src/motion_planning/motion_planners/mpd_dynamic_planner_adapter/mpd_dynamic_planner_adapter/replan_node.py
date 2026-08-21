"""ROS 2 Phase-4 dynamic resident-MPD replanner."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from manipulation_motion_planning.contracts import (
    JointTarget,
    PoseTarget,
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)
from mpd_planner_adapter.backend import EXPECTED_JOINT_NAMES
from mpd_planner_adapter.coordinator import LatestOnlyPlanner
from mpd_planner_adapter.execution import JtcHandoffManager
from mpd_planner_adapter.trajectory import TimedPlan

from .backend import DynamicMpdGlobalTrajectoryBackend
from .braking import make_braking_plan
from .collision_guard import (
    DynamicTrajectoryGuard,
    TimedCollisionPlan,
    collision_plan_from_result,
    extend_collision_plan_with_terminal_hold,
    splice_collision_plans,
)
from .dynamic_world import DynamicWorldError, DynamicWorldManager, DynamicWorldSnapshot
from .candidate_selector import (
    CandidateCost,
    choose_hysteretic_switch,
    clearance_cost,
    common_window_kinematic_cost,
)
from .quintic_bridge import (
    QuinticBridgeError,
    predict_point_with_terminal_hold,
    select_quintic_handoff,
    splice_with_quintic_bridge,
)
from .replay_recorder import DynamicReplayRecorder


@dataclass(frozen=True)
class DynamicPlanningJob:
    generation: int
    world: DynamicWorldSnapshot
    start: StartState
    target: PoseTarget | JointTarget
    q_acc_start: tuple[float, ...]
    submitted_unix_ns: int
    bridge_start_unix_ns: int
    handoff_unix_ns: int
    deadline_unix_ns: int


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _duration(seconds: float):
    from builtin_interfaces.msg import Duration

    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


def _time(unix_ns: int):
    from builtin_interfaces.msg import Time

    return Time(sec=unix_ns // 1_000_000_000, nanosec=unix_ns % 1_000_000_000)


def _parse_target(text: str) -> PoseTarget | None:
    if not text.strip():
        return None
    values = [float(value) for value in text.replace(",", " ").split()]
    if len(values) != 7 or not all(math.isfinite(value) for value in values):
        raise ValueError("target_pose_xyzw must be empty or seven comma/space-separated numbers")
    x, y, z, qx, qy, qz, qw = values
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        raise ValueError("target quaternion has zero norm")
    return PoseTarget((x, y, z), (qw / norm, qx / norm, qy / norm, qz / norm))


def _prepend_execution_prefix(
    active_plan: TimedPlan | None,
    selected: TrajectoryPlanResult,
    *,
    monitoring_start_unix_s: float,
    bridge_start_unix_s: float,
    sample_dt_s: float,
) -> TrajectoryPlanResult:
    duration = bridge_start_unix_s - monitoring_start_unix_s
    if duration <= 0.0 or sample_dt_s <= 0.0:
        raise ValueError("execution prefix interval is invalid")
    count = max(2, int(math.ceil(duration / sample_dt_s)) + 1)
    absolute_times = np.linspace(monitoring_start_unix_s, bridge_start_unix_s, count)
    if active_plan is None:
        first = selected.points[0]
        prefix = [
            TrajectoryPlanPoint(
                positions=list(first.positions),
                velocities=list(first.velocities or np.zeros(len(first.positions))),
                accelerations=list(first.accelerations or np.zeros(len(first.positions))),
                time_from_start_s=float(stamp - monitoring_start_unix_s),
            )
            for stamp in absolute_times
        ]
    else:
        prefix = []
        for stamp in absolute_times:
            point = predict_point_with_terminal_hold(active_plan, float(stamp))
            prefix.append(
                TrajectoryPlanPoint(
                    positions=list(point.positions),
                    velocities=list(point.velocities or np.zeros(len(point.positions))),
                    accelerations=list(point.accelerations or np.zeros(len(point.positions))),
                    time_from_start_s=float(stamp - monitoring_start_unix_s),
                )
            )
    merged = prefix + [
        TrajectoryPlanPoint(
            positions=list(point.positions),
            velocities=None if point.velocities is None else list(point.velocities),
            accelerations=None if point.accelerations is None else list(point.accelerations),
            time_from_start_s=duration + point.time_from_start_s,
        )
        for point in selected.points[1:]
    ]
    return TrajectoryPlanResult(
        valid=True,
        joint_names=list(selected.joint_names or []),
        points=merged,
        diagnostics=dict(selected.diagnostics),
    )


class MpdDynamicReplanNode(Node):
    def __init__(self) -> None:
        super().__init__("mpd_dynamic_replanner")
        defaults = {
            "socket_path": "/tmp/mpd-dynamic-runtime.sock",
            "scene_id": "EnvWarehouseExtraObjectsV00",
            "seed": 123,
            "worker_timeout_s": 3.0,
            "plan_rate_hz": 1.0,
            "planning_budget_s": 1.4,
            "commit_margin_s": 0.15,
            "handoff_search_horizon_s": 4.0,
            "handoff_step_s": 0.05,
            "trajectory_duration_s": 10.0,
            "prediction_horizon_s": 15.0,
            "max_state_age_s": 0.25,
            "max_world_age_s": 0.25,
            "plan_only": True,
            "joint_state_topic": "/franka/joint_states",
            "world_observation_topic": "/mpd/dynamic_world_observations",
            "planned_trajectory_topic": "~/planned_trajectory",
            "target_pose_xyzw": "",
            "jtc_action_name": "/franka_arm_jtc/follow_joint_trajectory",
            "command_lead_s": 0.05,
            "prefix_dt_s": 0.05,
            "max_start_drift_rad": 0.10,
            "enforce_measured_start_drift": True,
            "max_handoff_speed_rad_s": 0.20,
            "max_q_jump_rad": 0.03,
            "max_dq_jump_rad_s": 0.20,
            "max_ddq_jump_rad_s2": 2.0,
            "bridge_minimum_duration_s": 0.20,
            "bridge_sample_dt_s": 0.02,
            "bridge_max_velocity_rad_s": 1.5,
            "bridge_max_acceleration_rad_s2": 3.0,
            "bridge_max_jerk_rad_s3": 15.0,
            "bridge_max_active_deviation_rad": 0.08,
            "comparison_horizon_s": 2.0,
            "comparison_sample_dt_s": 0.02,
            "preferred_clearance_m": 0.10,
            "cost_kinematic_weight": 1.0,
            "cost_tail_kinematic_weight": 1.0,
            "cost_clearance_weight": 4.0,
            "cost_mpd_weight": 0.10,
            "cost_bridge_weight": 0.10,
            "cost_switch_penalty": 0.02,
            "switching_hysteresis": 0.02,
            "minimum_commit_interval_s": 1.0,
            "replacement_retry_reserve_s": 3.0,
            "enable_exhaustion_forced_switch": False,
            "guard_rate_hz": 20.0,
            "guard_lookahead_s": 2.0,
            "guard_check_dt_s": 0.02,
            "guard_minimum_clearance_m": 0.0,
            "covariance_sigma": 3.0,
            "process_acceleration_std_m_s2": 0.01,
            "max_dynamic_objects": 16,
            "brake_max_deceleration_rad_s2": 1.0,
            "brake_minimum_duration_s": 0.20,
            "brake_sample_dt_s": 0.02,
            "replay_record_dir": "",
            "replay_env_name": "",
            "replay_static_scene_json": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        value = lambda name: self.get_parameter(name).value
        self._planning_budget_s = float(value("planning_budget_s"))
        self._commit_margin_s = float(value("commit_margin_s"))
        self._handoff_search_horizon_s = float(value("handoff_search_horizon_s"))
        self._handoff_step_s = float(value("handoff_step_s"))
        self._trajectory_duration_s = float(value("trajectory_duration_s"))
        self._max_state_age_s = float(value("max_state_age_s"))
        self._max_world_age_s = float(value("max_world_age_s"))
        self._plan_only = bool(value("plan_only"))
        self._command_lead_s = float(value("command_lead_s"))
        self._guard_lookahead_s = float(value("guard_lookahead_s"))
        plan_rate_hz, guard_rate_hz = float(value("plan_rate_hz")), float(value("guard_rate_hz"))
        if plan_rate_hz <= 0.0 or guard_rate_hz <= 0.0:
            raise ValueError("planner and guard rates must be positive")
        if self._planning_budget_s <= self._commit_margin_s:
            raise ValueError("planning_budget_s must exceed commit_margin_s")
        self._splice_options = {
            "prefix_dt_s": float(value("prefix_dt_s")),
            "max_start_drift_rad": float(value("max_start_drift_rad")),
            "max_handoff_speed_rad_s": float(value("max_handoff_speed_rad_s")),
            "max_q_jump_rad": float(value("max_q_jump_rad")),
            "max_dq_jump_rad_s": float(value("max_dq_jump_rad_s")),
            "max_ddq_jump_rad_s2": float(value("max_ddq_jump_rad_s2")),
        }
        self._enforce_measured_start_drift = bool(value("enforce_measured_start_drift"))
        self._bridge_options = {
            "minimum_duration_s": float(value("bridge_minimum_duration_s")),
            "sample_dt_s": float(value("bridge_sample_dt_s")),
            "max_velocity_rad_s": float(value("bridge_max_velocity_rad_s")),
            "max_acceleration_rad_s2": float(value("bridge_max_acceleration_rad_s2")),
            "max_jerk_rad_s3": float(value("bridge_max_jerk_rad_s3")),
        }
        self._bridge_max_active_deviation_rad = float(
            value("bridge_max_active_deviation_rad")
        )
        self._comparison_horizon_s = float(value("comparison_horizon_s"))
        self._comparison_sample_dt_s = float(value("comparison_sample_dt_s"))
        self._preferred_clearance_m = float(value("preferred_clearance_m"))
        self._cost_weights = {
            "kinematic": float(value("cost_kinematic_weight")),
            "tail_kinematic": float(value("cost_tail_kinematic_weight")),
            "clearance": float(value("cost_clearance_weight")),
            "mpd": float(value("cost_mpd_weight")),
            "bridge": float(value("cost_bridge_weight")),
            "switch": float(value("cost_switch_penalty")),
        }
        self._switching_hysteresis = float(value("switching_hysteresis"))
        self._minimum_commit_interval_s = float(value("minimum_commit_interval_s"))
        self._replacement_retry_reserve_s = float(value("replacement_retry_reserve_s"))
        self._enable_exhaustion_forced_switch = bool(
            value("enable_exhaustion_forced_switch")
        )
        if any(option <= 0.0 for option in self._bridge_options.values()):
            raise ValueError("quintic bridge durations, sampling, and limits must be positive")
        if (
            self._comparison_horizon_s <= 0.0
            or self._comparison_horizon_s >= self._trajectory_duration_s
            or self._comparison_sample_dt_s <= 0.0
            or self._minimum_commit_interval_s < 0.0
            or self._replacement_retry_reserve_s < 0.0
            or self._switching_hysteresis < 0.0
            or self._bridge_max_active_deviation_rad <= 0.0
        ):
            raise ValueError("dynamic handoff comparison parameters are invalid")
        if any(weight < 0.0 for weight in self._cost_weights.values()):
            raise ValueError("dynamic handoff cost weights must be non-negative")
        self._brake_options = {
            "max_deceleration_rad_s2": float(value("brake_max_deceleration_rad_s2")),
            "minimum_duration_s": float(value("brake_minimum_duration_s")),
            "sample_dt_s": float(value("brake_sample_dt_s")),
        }
        covariance_sigma = float(value("covariance_sigma"))
        process_std = float(value("process_acceleration_std_m_s2"))
        self._world_manager = DynamicWorldManager(
            prediction_horizon_s=float(value("prediction_horizon_s")),
            max_objects=int(value("max_dynamic_objects")),
            process_acceleration_std_m_s2=process_std,
        )
        self._guard = DynamicTrajectoryGuard(
            check_dt_s=float(value("guard_check_dt_s")),
            covariance_sigma=covariance_sigma,
            process_acceleration_std_m_s2=process_std,
            minimum_clearance_m=float(value("guard_minimum_clearance_m")),
        )
        self._backend = DynamicMpdGlobalTrajectoryBackend(
            str(value("socket_path")),
            scene_id=str(value("scene_id")),
            seed=int(value("seed")),
            timeout_s=float(value("worker_timeout_s")),
        )
        self._backend_ready = False
        self._planner = LatestOnlyPlanner(self._plan_job)
        self._generation = time.time_ns()
        self._state: StartState | None = None
        self._state_received_monotonic = 0.0
        self._world_received_monotonic = 0.0
        self._target: PoseTarget | JointTarget | None = _parse_target(str(value("target_pose_xyzw")))
        self._active_plan: TimedPlan | None = None
        self._active_collision_plan: TimedCollisionPlan | None = None
        self._candidate_plans: dict[int, tuple[TimedPlan, TimedCollisionPlan | None]] = {}
        self._emergency_stopped = False
        self._braking = False
        self._last_guard_reason = "not_checked"
        self._last_guard_clearance_m = math.inf
        self._last_commit_unix_s = -math.inf
        self._last_switch_decision = "not_evaluated"
        self._last_old_cost = math.inf
        self._last_new_cost = math.inf
        replay_record_dir = str(value("replay_record_dir"))
        self._replay_recorder = (
            DynamicReplayRecorder(
                replay_record_dir,
                env_name=str(value("replay_env_name")) or str(value("scene_id")),
                static_scene_path=str(value("replay_static_scene_json")),
            )
            if replay_record_dir
            else None
        )
        self._replay_record_error_logged = False
        self._latencies_s: deque[float] = deque(maxlen=512)
        self._counters = {
            key: 0
            for key in (
                "submitted",
                "accepted",
                "invalid",
                "superseded",
                "deadline_miss",
                "worker_error",
                "handoff_rejected",
                "world_revalidation_rejected",
                "guard_brakes",
                "no_handoff_brakes",
                "no_bridge_retry",
                "hysteresis_kept_old",
                "minimum_interval_kept_old",
                "goal_submitted",
                "goal_accepted",
                "goal_terminal",
            )
        }

        self._trajectory_publisher = self.create_publisher(
            JointTrajectory, str(value("planned_trajectory_topic")), 1
        )
        self._diagnostics_publisher = self.create_publisher(String, "~/diagnostics", 10)
        self._execution = (
            None
            if self._plan_only
            else JtcHandoffManager(
                self,
                str(value("jtc_action_name")),
                on_accepted=self._on_goal_accepted,
                on_terminal=self._on_goal_terminal,
            )
        )
        self.create_subscription(JointState, str(value("joint_state_topic")), self._on_joint_state, 10)
        self.create_subscription(String, str(value("world_observation_topic")), self._on_world, 1)
        self.create_subscription(PoseStamped, "~/pose_target", self._on_pose_target, 1)
        self.create_subscription(JointState, "~/joint_target", self._on_joint_target, 1)
        self.create_subscription(Bool, "~/stop", self._on_stop, 1)
        stop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, "/mpd/emergency_stop", self._on_emergency_stop, stop_qos
        )
        self.create_service(Trigger, "~/safe_stop", self._on_safe_stop)
        self.create_service(Trigger, "~/reset_stop", self._on_reset_stop)
        self.create_timer(1.0 / plan_rate_hz, self._schedule)
        self.create_timer(0.02, self._drain)
        self.create_timer(1.0 / guard_rate_hz, self._guard_active_plan)
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            f"dynamic MPD replanner started (plan_only={self._plan_only}, rate={plan_rate_hz:.2f} Hz)"
        )

    def _invalidate(self) -> None:
        self._generation += 1
        self._planner.invalidate(self._generation)

    def _record_replay(self, method: str, *args, **kwargs):
        """Keep optional replay I/O strictly outside the planning safety path."""
        recorder = self._replay_recorder
        if recorder is None:
            return None
        try:
            result = getattr(recorder, method)(*args, **kwargs)
            if method in {
                "record_candidate",
                "record_rejected",
                "record_activation",
                "record_terminal",
            }:
                recorder.flush()
            return result
        except Exception as error:  # Replay must never perturb planning/execution.
            if not self._replay_record_error_logged:
                self.get_logger().error(f"disabled dynamic replay recording: {error}")
                self._replay_record_error_logged = True
            self._replay_recorder = None
            return None

    def _on_joint_state(self, message: JointState) -> None:
        try:
            positions = dict(zip(message.name, message.position))
            velocities = dict(zip(message.name, message.velocity))
            q = [float(positions[name]) for name in EXPECTED_JOINT_NAMES]
            dq = [float(velocities.get(name, 0.0)) for name in EXPECTED_JOINT_NAMES]
            if not all(math.isfinite(item) for item in q + dq):
                raise ValueError("non-finite state")
        except (KeyError, ValueError) as error:
            self.get_logger().warning(f"ignored incomplete joint state: {error}")
            return
        self._state = StartState(list(EXPECTED_JOINT_NAMES), q, dq, _stamp_s(message.header.stamp))
        self._state_received_monotonic = time.monotonic()
        self._record_replay("record_state", self._state)

    def _on_world(self, message: String) -> None:
        try:
            observation = json.loads(message.data)
            snapshot = self._world_manager.update(observation)
        except (json.JSONDecodeError, DynamicWorldError, TypeError, ValueError) as error:
            self.get_logger().warning(f"ignored invalid dynamic world: {error}")
            return
        self._world_received_monotonic = time.monotonic()
        self._record_replay("record_world", snapshot)
        self.get_logger().debug(f"accepted dynamic world version {snapshot.version}")

    def _on_pose_target(self, message: PoseStamped) -> None:
        p, q = message.pose.position, message.pose.orientation
        norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
        if not math.isfinite(norm) or norm < 1e-9:
            return
        self._target = PoseTarget(
            (p.x, p.y, p.z),
            (q.w / norm, q.x / norm, q.y / norm, q.z / norm),
            _stamp_s(message.header.stamp),
        )
        self._braking = False
        self._invalidate()

    def _on_joint_target(self, message: JointState) -> None:
        try:
            by_name = dict(zip(message.name, message.position))
            target = [float(by_name[name]) for name in EXPECTED_JOINT_NAMES]
        except (KeyError, ValueError):
            return
        if not all(math.isfinite(item) for item in target):
            return
        self._target = JointTarget(list(EXPECTED_JOINT_NAMES), target, _stamp_s(message.header.stamp))
        self._braking = False
        self._invalidate()

    def _on_stop(self, message: Bool) -> None:
        if message.data:
            self._controlled_brake("stop_topic", clear_target=True)

    def _on_emergency_stop(self, message: Bool) -> None:
        if not message.data:
            return
        self._emergency_stopped = True
        self._target = None
        self._active_plan = None
        self._active_collision_plan = None
        self._invalidate()
        if self._execution is not None:
            self._execution.cancel()
        self.get_logger().error("emergency stop latched; reset_stop is required")

    def _on_safe_stop(self, _request, response):
        self._controlled_brake("safe_stop_service", clear_target=True)
        response.success = True
        response.message = "controlled braking requested"
        return response

    def _on_reset_stop(self, _request, response):
        if self._state is None or self._world_manager.snapshot is None:
            response.success = False
            response.message = "fresh robot state and dynamic world are required"
            return response
        self._emergency_stopped = False
        self._braking = False
        response.success = True
        response.message = "dynamic replanner stop latch reset; publish a target"
        return response

    def _plan_job(self, job: DynamicPlanningJob):
        if not self._backend_ready:
            self._backend.warmup()
            self._backend_ready = True
        if self._backend.uploaded_world_version != job.world.version:
            self._backend.update_world_snapshot(job.world)
        return self._backend.plan(
            job.start,
            job.target,
            {
                "request_seq": job.generation,
                "request_id": f"ros-dynamic-generation-{job.generation}",
                "world_version": job.world.version,
                "handoff_unix_ns": job.handoff_unix_ns,
                "deadline_unix_ns": job.deadline_unix_ns,
                "q_acc_start": list(job.q_acc_start),
            },
        )

    def _schedule(self) -> None:
        world = self._world_manager.snapshot
        if self._emergency_stopped or self._braking or self._state is None or self._target is None or world is None:
            return
        if self._planner.active or self._planner.pending_count:
            return
        if time.monotonic() - self._state_received_monotonic > self._max_state_age_s:
            return
        if time.monotonic() - self._world_received_monotonic > self._max_world_age_s:
            self._controlled_brake("stale_dynamic_world", clear_target=True)
            return
        now = time.time()
        planning_deadline = now + self._planning_budget_s
        bridge_start = planning_deadline + max(
            self._command_lead_s, self._commit_margin_s
        )
        latest_for_new_horizon = world.valid_until_unix_ns * 1e-9 - self._trajectory_duration_s
        handoff = None
        bridge_duration = None
        if self._active_plan is not None and self._active_collision_plan is not None:
            latest_handoff = min(
                now + self._handoff_search_horizon_s, latest_for_new_horizon
            )
            handoff_collision_plan = extend_collision_plan_with_terminal_hold(
                self._active_collision_plan, latest_handoff
            )
            choice = select_quintic_handoff(
                active_plan=self._active_plan,
                collision_plan=handoff_collision_plan,
                world=world,
                guard=self._guard,
                now_unix_s=now,
                bridge_start_unix_s=bridge_start,
                latest_handoff_unix_s=latest_handoff,
                step_s=self._handoff_step_s,
                allow_terminal_hold=True,
                **self._bridge_options,
            )
            handoff = choice.handoff_unix_s
            bridge_duration = choice.bridge_duration_s
        else:
            bridge_duration = self._bridge_options["minimum_duration_s"]
            candidate_handoff = bridge_start + bridge_duration
            if candidate_handoff <= latest_for_new_horizon:
                handoff = candidate_handoff
        if handoff is None:
            self._counters["no_bridge_retry"] += 1
            self._last_switch_decision = "no_dynamic_safe_quintic_bridge_retry"
            # A failed replacement search is not itself a safety event.  The
            # independent 20 Hz guard owns braking while the old command remains safe.
            return
        try:
            if self._active_plan is not None:
                handoff_point = predict_point_with_terminal_hold(
                    self._active_plan, handoff
                )
                start = StartState(
                    list(self._active_plan.result.joint_names or []),
                    list(handoff_point.positions),
                    list(handoff_point.velocities or np.zeros(7)),
                    handoff,
                )
                q_acc_start = tuple(handoff_point.accelerations or np.zeros(7))
            else:
                start = StartState(
                    self._state.joint_names,
                    self._state.positions,
                    self._state.velocities,
                    handoff,
                )
                q_acc_start = tuple(np.zeros(7))
        except ValueError as error:
            self.get_logger().warning(f"handoff prediction failed: {error}")
            return
        handoff_ns = int(handoff * 1e9)
        self._generation += 1
        job = DynamicPlanningJob(
            self._generation,
            world,
            start,
            self._target,
            q_acc_start,
            int(now * 1e9),
            int(bridge_start * 1e9),
            handoff_ns,
            int(planning_deadline * 1e9),
        )
        self._planner.submit(job.generation, job)
        self._counters["submitted"] += 1

    def _drain(self) -> None:
        for completion in self._planner.drain():
            if completion.superseded or completion.generation != self._generation:
                self._counters["superseded"] += 1
                if completion.result is not None and completion.result.valid:
                    self._record_replay(
                        "record_rejected",
                        completion.generation,
                        completion.result,
                        start_unix_s=completion.job.handoff_unix_ns * 1e-9,
                    )
                continue
            if completion.error is not None:
                self._counters["worker_error"] += 1
                self.get_logger().error(f"dynamic planning failed: {completion.error}")
                continue
            self._latencies_s.append(completion.elapsed_s)
            result = completion.result
            if result is None or not result.valid:
                self._counters["invalid"] += 1
                self.get_logger().warning(f"dynamic plan rejected: {None if result is None else result.reason}")
                continue
            if time.time_ns() >= completion.job.deadline_unix_ns:
                self._counters["deadline_miss"] += 1
                self._record_replay(
                    "record_rejected",
                    completion.generation,
                    result,
                    start_unix_s=completion.job.handoff_unix_ns * 1e-9,
                )
                continue
            if not self._commit(completion.job, result):
                continue
            self._counters["accepted"] += 1

    def _commit(self, job: DynamicPlanningJob, result) -> bool:
        latest_world = self._world_manager.snapshot
        if latest_world is None or self._state is None:
            return False
        handoff = job.handoff_unix_ns * 1e-9
        bridge_start = job.bridge_start_unix_ns * 1e-9
        if self._plan_only:
            new_collision = collision_plan_from_result(result, handoff)
            try:
                risk = self._guard.validate(
                    new_collision,
                    latest_world,
                    handoff,
                    float(new_collision.absolute_times_s[-1]),
                )
            except ValueError as error:
                self._counters["world_revalidation_rejected"] += 1
                self.get_logger().warning(
                    f"plan-only latest-world validation failed: {error}"
                )
                return False
            current_world = self._world_manager.snapshot
            if (
                not risk.safe
                or current_world is None
                or current_world.version != latest_world.version
            ):
                self._counters["world_revalidation_rejected"] += 1
                return False
            # plan_only does not pretend that the unexecuted trajectory is the
            # robot's active command.  Every cycle starts from measured state.
            self._trajectory_publisher.publish(
                self._to_message(result, job.handoff_unix_ns)
            )
            self._record_replay(
                "record_candidate",
                job.generation,
                result,
                start_unix_s=handoff,
                handoff_unix_s=handoff,
            )
            return True

        now = time.time()
        if bridge_start <= now + self._command_lead_s:
            self._counters["deadline_miss"] += 1
            self._last_switch_decision = "bridge_start_deadline_miss"
            return False
        try:
            if self._active_plan is None:
                bridge_initial = StartState(
                    list(self._state.joint_names),
                    list(self._state.positions),
                    list(self._state.velocities or np.zeros(7)),
                    bridge_start,
                )
                bridge_initial_acceleration = np.zeros(7)
            else:
                expected_now = predict_point_with_terminal_hold(
                    self._active_plan, now
                )
                drift = float(
                    np.max(
                        np.abs(
                            np.asarray(self._state.positions)
                            - np.asarray(expected_now.positions)
                        )
                    )
                )
                if (
                    self._enforce_measured_start_drift
                    and drift > self._splice_options["max_start_drift_rad"]
                ):
                    raise QuinticBridgeError(
                        f"active-plan drift {drift:.6f} rad exceeds configured limit"
                    )
                initial_point = predict_point_with_terminal_hold(
                    self._active_plan, bridge_start
                )
                bridge_initial = StartState(
                    list(self._active_plan.result.joint_names or []),
                    list(initial_point.positions),
                    list(initial_point.velocities or np.zeros(7)),
                    bridge_start,
                )
                bridge_initial_acceleration = np.asarray(
                    initial_point.accelerations or np.zeros(7), dtype=np.float64
                )
        except (QuinticBridgeError, ValueError) as error:
            self._counters["handoff_rejected"] += 1
            self.get_logger().warning(f"dynamic handoff rejected: {error}")
            self._record_replay(
                "record_rejected",
                job.generation,
                result,
                start_unix_s=handoff,
            )
            return False

        candidates = result.diagnostics.get("top_k_candidates", [result])
        if not isinstance(candidates, list) or not candidates:
            self._counters["invalid"] += 1
            return False
        raw_scores = np.asarray(
            [candidate.diagnostics.get("mpd_selection_score", 0.0) for candidate in candidates],
            dtype=np.float64,
        )
        score_range = float(np.ptp(raw_scores))
        normalized_scores = (
            np.zeros_like(raw_scores)
            if score_range <= 1e-12
            else (raw_scores - float(raw_scores.min())) / score_range
        )

        selected = None
        selected_collision = None
        selected_risk = None
        selected_decision = None
        old_safe = self._active_plan is None
        for _attempt in range(2):
            evaluation_world = self._world_manager.snapshot
            if evaluation_world is None:
                return False
            candidate_costs = []
            candidate_artifacts = {}
            for index, candidate in enumerate(candidates):
                try:
                    merged = splice_with_quintic_bridge(
                        current_state=bridge_initial,
                        current_acceleration=bridge_initial_acceleration,
                        new_plan=candidate,
                        duration_s=handoff - bridge_start,
                        sample_dt_s=self._bridge_options["sample_dt_s"],
                        max_velocity_rad_s=self._bridge_options["max_velocity_rad_s"],
                        max_acceleration_rad_s2=self._bridge_options[
                            "max_acceleration_rad_s2"
                        ],
                        max_jerk_rad_s3=self._bridge_options["max_jerk_rad_s3"],
                    )
                    if self._active_plan is not None:
                        bridge_points = [
                            point
                            for point in merged.points
                            if point.time_from_start_s <= handoff - bridge_start + 1e-9
                        ]
                        bridge_deviation = max(
                            float(
                                np.max(
                                    np.abs(
                                        np.asarray(point.positions)
                                        - np.asarray(
                                            predict_point_with_terminal_hold(
                                                self._active_plan,
                                                bridge_start
                                                + point.time_from_start_s,
                                            ).positions
                                        )
                                    )
                                )
                            )
                            for point in bridge_points
                        )
                        if bridge_deviation > self._bridge_max_active_deviation_rad:
                            raise QuinticBridgeError(
                                f"bridge/active deviation {bridge_deviation:.6f} rad exceeds limit"
                            )
                    else:
                        bridge_deviation = 0.0
                    new_collision = collision_plan_from_result(candidate, handoff)
                    active_collision = (
                        None
                        if self._active_collision_plan is None
                        else extend_collision_plan_with_terminal_hold(
                            self._active_collision_plan, handoff
                        )
                    )
                    merged_collision = splice_collision_plans(
                        active_collision,
                        new_collision,
                        bridge_start,
                        handoff,
                        self._splice_options["prefix_dt_s"],
                    )
                    monitoring_collision = splice_collision_plans(
                        active_collision,
                        merged_collision,
                        now,
                        bridge_start,
                        self._splice_options["prefix_dt_s"],
                    )
                    risk = self._guard.validate(
                        monitoring_collision,
                        evaluation_world,
                        now,
                        float(monitoring_collision.absolute_times_s[-1]),
                    )
                    if not risk.safe:
                        continue
                    # Compare both alternatives from the identical handoff state.
                    # The bridge is scored separately, while this first window and
                    # the following tail cover the complete future command horizon.
                    window_end = handoff + self._comparison_horizon_s
                    comparison_end = handoff + self._trajectory_duration_s
                    kinematic = common_window_kinematic_cost(
                        merged,
                        trajectory_start_unix_s=bridge_start,
                        window_start_unix_s=handoff,
                        window_end_unix_s=window_end,
                        max_velocity_rad_s=self._bridge_options["max_velocity_rad_s"],
                        max_acceleration_rad_s2=self._bridge_options[
                            "max_acceleration_rad_s2"
                        ],
                        max_jerk_rad_s3=self._bridge_options["max_jerk_rad_s3"],
                        sample_dt_s=self._comparison_sample_dt_s,
                        hold_after_end=True,
                    )
                    tail_kinematic = common_window_kinematic_cost(
                        merged,
                        trajectory_start_unix_s=bridge_start,
                        window_start_unix_s=window_end,
                        window_end_unix_s=comparison_end,
                        max_velocity_rad_s=self._bridge_options["max_velocity_rad_s"],
                        max_acceleration_rad_s2=self._bridge_options[
                            "max_acceleration_rad_s2"
                        ],
                        max_jerk_rad_s3=self._bridge_options["max_jerk_rad_s3"],
                        sample_dt_s=self._comparison_sample_dt_s,
                        hold_after_end=True,
                    )
                    comparison_collision = extend_collision_plan_with_terminal_hold(
                        new_collision, comparison_end
                    )
                    comparison_risk = self._guard.validate(
                        comparison_collision,
                        evaluation_world,
                        handoff,
                        comparison_end,
                    )
                    if not comparison_risk.safe:
                        continue
                    clearance = clearance_cost(
                        comparison_risk.minimum_clearance_m,
                        self._preferred_clearance_m,
                    )
                    bridge_stats = merged.diagnostics["bridge"]
                    bridge_cost = (
                        float(bridge_stats["duration_s"])
                        / max(self._comparison_horizon_s, 1e-9)
                        + float(bridge_stats["velocity_utilization"])
                        + float(bridge_stats["acceleration_utilization"])
                        + float(bridge_stats["jerk_utilization"])
                    ) / 4.0
                    total = (
                        self._cost_weights["kinematic"] * kinematic
                        + self._cost_weights["tail_kinematic"] * tail_kinematic
                        + self._cost_weights["clearance"] * clearance
                        + self._cost_weights["mpd"] * float(normalized_scores[index])
                        + self._cost_weights["bridge"] * bridge_cost
                        + (self._cost_weights["switch"] if self._active_plan is not None else 0.0)
                    )
                    candidate_costs.append(
                        CandidateCost(
                            index,
                            total,
                            kinematic,
                            clearance,
                            float(normalized_scores[index]),
                            bridge_cost,
                            tail_kinematic,
                        )
                    )
                    merged.diagnostics.update(
                        bridge_active_deviation_rad=bridge_deviation,
                        composite_cost={
                            "total": total,
                            "kinematic": kinematic,
                            "tail_kinematic": tail_kinematic,
                            "clearance": clearance,
                            "mpd": float(normalized_scores[index]),
                            "bridge": bridge_cost,
                        },
                    )
                    candidate_artifacts[index] = (merged, monitoring_collision, risk)
                except (KeyError, QuinticBridgeError, ValueError):
                    continue

            old_cost = math.inf
            old_safe = False
            if self._active_plan is not None and self._active_collision_plan is not None:
                window_end = handoff + self._comparison_horizon_s
                comparison_end = handoff + self._trajectory_duration_s
                try:
                    old_comparison_collision = extend_collision_plan_with_terminal_hold(
                        self._active_collision_plan, comparison_end
                    )
                    old_risk = self._guard.validate(
                        old_comparison_collision,
                        evaluation_world,
                        handoff,
                        comparison_end,
                    )
                    old_kinematic = common_window_kinematic_cost(
                        self._active_plan.result,
                        trajectory_start_unix_s=self._active_plan.start_unix_s,
                        window_start_unix_s=handoff,
                        window_end_unix_s=window_end,
                        max_velocity_rad_s=self._bridge_options["max_velocity_rad_s"],
                        max_acceleration_rad_s2=self._bridge_options[
                            "max_acceleration_rad_s2"
                        ],
                        max_jerk_rad_s3=self._bridge_options["max_jerk_rad_s3"],
                        sample_dt_s=self._comparison_sample_dt_s,
                        hold_after_end=True,
                    )
                    old_tail_kinematic = common_window_kinematic_cost(
                        self._active_plan.result,
                        trajectory_start_unix_s=self._active_plan.start_unix_s,
                        window_start_unix_s=window_end,
                        window_end_unix_s=comparison_end,
                        max_velocity_rad_s=self._bridge_options["max_velocity_rad_s"],
                        max_acceleration_rad_s2=self._bridge_options[
                            "max_acceleration_rad_s2"
                        ],
                        max_jerk_rad_s3=self._bridge_options["max_jerk_rad_s3"],
                        sample_dt_s=self._comparison_sample_dt_s,
                        hold_after_end=True,
                    )
                    old_safe = (
                        old_risk.safe
                        and math.isfinite(old_kinematic)
                        and math.isfinite(old_tail_kinematic)
                    )
                    if old_safe:
                        old_cost = (
                            self._cost_weights["kinematic"] * old_kinematic
                            + self._cost_weights["tail_kinematic"]
                            * old_tail_kinematic
                            + self._cost_weights["clearance"]
                            * clearance_cost(
                                old_risk.minimum_clearance_m,
                                self._preferred_clearance_m,
                            )
                        )
                except ValueError:
                    old_safe = False
            forced_switch_reason = None
            if (
                self._enable_exhaustion_forced_switch
                and self._active_plan is not None
                and old_safe
            ):
                old_remaining_s = (
                    self._active_plan.start_unix_s
                    + self._active_plan.result.points[-1].time_from_start_s
                    - bridge_start
                )
                if old_remaining_s < (
                    self._comparison_horizon_s + self._replacement_retry_reserve_s
                ):
                    forced_switch_reason = "old_trajectory_exhaustion_reserve"
            decision = choose_hysteretic_switch(
                candidate_costs,
                old_cost=old_cost,
                old_safe=old_safe,
                minimum_commit_interval_elapsed=(
                    self._active_plan is None
                    or bridge_start - self._last_commit_unix_s
                    >= self._minimum_commit_interval_s
                ),
                switching_hysteresis=self._switching_hysteresis,
                forced_switch_reason=forced_switch_reason,
            )
            current_world = self._world_manager.snapshot
            if current_world is None:
                return False
            if current_world.version != evaluation_world.version:
                continue
            selected_decision = decision
            if decision.candidate_index is not None:
                selected, selected_collision, selected_risk = candidate_artifacts[
                    decision.candidate_index
                ]
            break

        if selected_decision is None:
            self._counters["world_revalidation_rejected"] += 1
            self._last_switch_decision = "world_changed_during_top_k_revalidation"
            return False
        self._last_switch_decision = selected_decision.reason
        self._last_old_cost = selected_decision.old_cost
        self._last_new_cost = selected_decision.new_cost
        if selected is None or selected_collision is None or selected_risk is None:
            if selected_decision.reason == "switching_hysteresis":
                self._counters["hysteresis_kept_old"] += 1
            elif selected_decision.reason == "minimum_commit_interval":
                self._counters["minimum_interval_kept_old"] += 1
            self._record_replay(
                "record_rejected",
                job.generation,
                result,
                start_unix_s=handoff,
            )
            if not old_safe and selected_decision.reason == "no_latest_world_safe_candidate":
                self._controlled_brake("no_latest_world_safe_candidate", clear_target=True)
            return False

        final_world = self._world_manager.snapshot
        if final_world is None:
            return False
        final_risk = self._guard.validate(
            selected_collision,
            final_world,
            now,
            float(selected_collision.absolute_times_s[-1]),
        )
        world_after_final_check = self._world_manager.snapshot
        if (
            not final_risk.safe
            or world_after_final_check is None
            or world_after_final_check.version != final_world.version
        ):
            self._counters["world_revalidation_rejected"] += 1
            self._last_switch_decision = "latest_world_final_revalidation_failed"
            if not old_safe:
                self._controlled_brake(
                    "latest_world_final_revalidation_failed", clear_target=True
                )
            return False

        selected.diagnostics["phase_timing"] = {
            "planning_submitted_unix_s": job.submitted_unix_ns * 1e-9,
            "bridge_start_unix_s": bridge_start,
            "handoff_unix_s": handoff,
            "old_continuation_s": bridge_start - job.submitted_unix_ns * 1e-9,
            "bridge_s": handoff - bridge_start,
            "mpd_suffix_s": selected.points[-1].time_from_start_s - (handoff - bridge_start),
        }
        selected.diagnostics["switch_decision"] = selected_decision.__dict__
        message = self._to_message(selected, job.bridge_start_unix_ns)
        self._trajectory_publisher.publish(message)
        if self._execution is None or not self._execution.submit(job.generation, message):
            self._counters["worker_error"] += 1
            return False
        monitored = _prepend_execution_prefix(
            self._active_plan,
            selected,
            monitoring_start_unix_s=now,
            bridge_start_unix_s=bridge_start,
            sample_dt_s=self._splice_options["prefix_dt_s"],
        )
        self._candidate_plans[job.generation] = (TimedPlan(monitored, now), selected_collision)
        self._record_replay(
            "record_candidate",
            job.generation,
            selected,
            start_unix_s=bridge_start,
            handoff_unix_s=handoff,
        )
        self._last_commit_unix_s = bridge_start
        self._counters["goal_submitted"] += 1
        return True

    def _guard_active_plan(self) -> None:
        if self._emergency_stopped or self._braking or self._active_collision_plan is None:
            return
        world = self._world_manager.snapshot
        if world is None:
            return
        now = time.time()
        end = min(now + self._guard_lookahead_s, world.valid_until_unix_ns * 1e-9)
        try:
            guarded_plan = extend_collision_plan_with_terminal_hold(
                self._active_collision_plan, end
            )
            risk = self._guard.validate(guarded_plan, world, now, end)
        except ValueError:
            return
        self._last_guard_clearance_m = risk.minimum_clearance_m
        self._last_guard_reason = "safe" if risk.safe else "dynamic_collision_or_prediction_invalid"
        if not risk.safe:
            self._counters["guard_brakes"] += 1
            self._controlled_brake(self._last_guard_reason, clear_target=True)

    def _controlled_brake(self, reason: str, *, clear_target: bool) -> None:
        if self._braking:
            return
        self._braking = True
        if clear_target:
            self._target = None
        self._invalidate()
        if self._state is None:
            if self._execution is not None:
                self._execution.cancel()
            return
        brake = make_braking_plan(self._state, **self._brake_options)
        start = time.time() + self._command_lead_s
        message = self._to_message(brake, int(start * 1e9))
        self._trajectory_publisher.publish(message)
        if self._execution is not None:
            self._generation += 1
            if self._execution.submit(self._generation, message):
                self._candidate_plans[self._generation] = (TimedPlan(brake, start), None)
                self._record_replay(
                    "record_candidate",
                    self._generation,
                    brake,
                    start_unix_s=start,
                    handoff_unix_s=None,
                    braking=True,
                    reason=reason,
                )
                self._counters["goal_submitted"] += 1
            else:
                self._execution.cancel()
        self.get_logger().error(f"controlled braking requested: {reason}")

    def _on_goal_accepted(self, plan_id: int) -> None:
        candidate = self._candidate_plans.get(plan_id)
        if candidate is not None:
            self._active_plan, self._active_collision_plan = candidate
        self._record_replay("record_activation", plan_id)
        self._counters["goal_accepted"] += 1

    def _on_goal_terminal(self, plan_id: int, state: str) -> None:
        self._candidate_plans.pop(plan_id, None)
        self._counters["goal_terminal"] += 1
        execution_idle = (
            self._execution is not None
            and self._execution.plan_id is None
            and self._execution.pending_plan_id is None
        )
        if (
            state == "SUCCEEDED"
            and execution_idle
        ):
            # A completed goal is a valid terminal hold, not an invitation to
            # generate another ten-second motion from the goal.  Keep its
            # collision occupancy guarded and wait for an explicitly new target.
            self._target = None
            self._invalidate()
        elif execution_idle:
            self._active_plan = None
            self._active_collision_plan = None
        if state in ("REJECTED", "ABORTED", "SEND_ERROR", "RESULT_ERROR"):
            self.get_logger().error(f"JTC dynamic plan {plan_id} entered {state}")
        self._record_replay("record_terminal", plan_id)

    @staticmethod
    def _to_message(result, start_unix_ns: int) -> JointTrajectory:
        message = JointTrajectory()
        message.header.stamp = _time(start_unix_ns)
        message.joint_names = list(result.joint_names or [])
        for point in result.points:
            output = JointTrajectoryPoint()
            output.positions = list(point.positions)
            if point.velocities is not None:
                output.velocities = list(point.velocities)
            if point.accelerations is not None:
                output.accelerations = list(point.accelerations)
            output.time_from_start = _duration(point.time_from_start_s)
            message.points.append(output)
        return message

    def _publish_diagnostics(self) -> None:
        world = self._world_manager.snapshot
        message = String()
        message.data = json.dumps(
            {
                "state": "EMERGENCY_STOPPED" if self._emergency_stopped else "READY",
                "generation": self._generation,
                "world_version": None if world is None else world.version,
                "worker_world_version": self._backend.uploaded_world_version,
                "planner_active": self._planner.active,
                "pending_count": self._planner.pending_count,
                "has_state": self._state is not None,
                "has_target": self._target is not None,
                "has_active_plan": self._active_plan is not None,
                "plan_only": self._plan_only,
                "braking": self._braking,
                "guard_reason": self._last_guard_reason,
                "guard_minimum_clearance_m": self._last_guard_clearance_m,
                "switch_decision": self._last_switch_decision,
                "old_common_window_cost": self._last_old_cost,
                "new_common_window_cost": self._last_new_cost,
                "latency_samples": len(self._latencies_s),
                **self._counters,
            },
            sort_keys=True,
        )
        self._diagnostics_publisher.publish(message)

    def destroy_node(self) -> bool:
        self._planner.close()
        if self._execution is not None:
            self._execution.destroy()
        if self._replay_recorder is not None:
            manifest_path = self._replay_recorder.close()
            if manifest_path is not None:
                self.get_logger().info(f"dynamic replay manifest: {manifest_path}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MpdDynamicReplanNode()
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
