"""ROS planner source nodes.

These nodes own scheduling, ROS subscriptions, validation, and command output.
Backends remain ROS-free solver adapters.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Optional

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .backend import GlobalSetpointBackend, GlobalTrajectoryBackend, OnlineMpcBackend
from .command_sink import CommandSink, make_command_sink
from .contracts import (
    HorizonPlanPoint,
    PoseTarget,
    StartState,
    Target,
    TrajectoryPlanResult,
    World,
)
from .robot_context import (
    CachedJointStateProvider,
    RobotContext,
    RobotModelInfo,
    StaticRobotModelProvider,
)


@dataclass(frozen=True)
class PlannerSourceConfig:
    """Common ROS source configuration."""

    source_name: str
    source_namespace: str = "/action_sources"
    state_topic: str = "/joint_states"
    command_sink_mode: str = "em"
    direct_command_topic: Optional[str] = None


@dataclass(frozen=True)
class GlobalSetpointSourceConfig(PlannerSourceConfig):
    """Configuration for streaming/requested global setpoint planning."""

    pose_topic: str = "/teleop/pose_commands"
    plan_rate_hz: float = 50.0
    max_state_age_s: float = 0.05
    max_target_age_s: float = 0.5
    plan_options: dict = field(default_factory=dict)


@dataclass(frozen=True)
class OnlineMpcSourceConfig(PlannerSourceConfig):
    """Configuration for online receding-horizon MPC."""

    step_rate_hz: float = 50.0
    max_state_age_s: float = 0.05
    max_target_age_s: float = 0.1


class GlobalSetpointPlannerSource(Node):
    """Pose target stream → global setpoint backend → command sink."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self._configured = False

    def configure(
        self,
        *,
        backend: GlobalSetpointBackend,
        state_joint_names: list[str],
        output_joint_names: list[str],
        config: GlobalSetpointSourceConfig,
        command_sink: CommandSink | None = None,
        robot_context: RobotContext | None = None,
    ) -> None:
        if self._configured:
            raise RuntimeError(f"{self.get_name()} is already configured")
        if config.plan_rate_hz <= 0.0:
            raise ValueError("plan_rate_hz must be positive")
        if not state_joint_names:
            raise ValueError("state_joint_names must not be empty")
        if not output_joint_names:
            raise ValueError("output_joint_names must not be empty")

        missing_output = [
            name for name in output_joint_names if name not in state_joint_names
        ]
        if missing_output:
            raise ValueError(
                f"output_joint_names not in state_joint_names: {missing_output}"
            )

        self._backend = backend
        self._state_provider = CachedJointStateProvider(
            state_joint_names, config.max_state_age_s
        )
        self._robot_context = robot_context or RobotContext(
            state_provider=self._state_provider,
            model_provider=StaticRobotModelProvider(
                RobotModelInfo(joint_names=list(state_joint_names))
            ),
        )
        self._output_joint_names = list(output_joint_names)
        self._config = config
        self._last_target: Optional[Target] = None
        self._last_target_stamp_s: Optional[float] = None
        self._success_count = 0
        self._fail_count = 0
        self._last_error: Optional[str] = None

        self._command_sink = command_sink or make_command_sink(
            self,
            mode=config.command_sink_mode,
            source_name=config.source_name,
            source_namespace=config.source_namespace,
            direct_command_topic=config.direct_command_topic,
        )
        self._diagnostics_pub = self.create_publisher(String, "~/diagnostics", 10)

        self.create_subscription(JointState, config.state_topic, self._on_state, 10)
        self.create_subscription(PoseStamped, config.pose_topic, self._on_pose, 10)

        self._backend.warmup()
        self.create_timer(1.0 / config.plan_rate_hz, self._on_plan_timer)
        self._configured = True

        self.get_logger().info(
            f"{self.get_name()}: global setpoint plan_rate={config.plan_rate_hz:.1f}Hz "
            f"source={config.source_namespace.rstrip('/')}/{config.source_name}"
        )

    def set_target(self, target: Target, stamp_s: float) -> None:
        self._last_target = target
        self._last_target_stamp_s = stamp_s

    def _on_state(self, msg: JointState) -> None:
        stamp_s = _stamp_or_now_s(self, msg)
        self._state_provider.update(
            list(msg.name),
            list(msg.position),
            list(msg.velocity) if msg.velocity else None,
            stamp_s,
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        position = msg.pose.position
        orientation = msg.pose.orientation
        target = PoseTarget(
            position_xyz=(position.x, position.y, position.z),
            orientation_wxyz=(
                orientation.w,
                orientation.x,
                orientation.y,
                orientation.z,
            ),
        )
        self.set_target(target, _stamp_or_now_s(self, msg))

    def _on_plan_timer(self) -> None:
        now_s = _now_s(self)
        current_state = self._robot_context.get_current_state(now_s)
        if current_state is None:
            self._note_failure(_state_error_reason(self._state_provider))
            return

        if self._last_target is None or self._last_target_stamp_s is None:
            self._note_failure("target_missing")
            return
        if (now_s - self._last_target_stamp_s) > self._config.max_target_age_s:
            self._note_failure("target_stale")
            return

        options = dict(self._config.plan_options)
        options.setdefault("dt", 1.0 / self._config.plan_rate_hz)

        try:
            result = self._backend.plan(current_state, self._last_target, options)
        except Exception as exc:  # noqa: BLE001
            self._note_failure(f"backend_exception:{exc}")
            return

        if not result.valid or result.joint_names is None or result.positions is None:
            self._note_failure(result.reason or "backend_invalid_result")
            return

        positions_by_name = dict(zip(result.joint_names, result.positions))
        try:
            output_positions = [
                float(positions_by_name[name]) for name in self._output_joint_names
            ]
        except KeyError as exc:
            self._note_failure(f"missing_joint_in_result:{exc}")
            return

        if not all(math.isfinite(v) for v in output_positions):
            self._note_failure("non_finite_positions")
            return

        self._command_sink.publish_joint_target(
            self, self._output_joint_names, output_positions
        )
        self._note_success(result.diagnostics)

    def _note_failure(self, reason: str) -> None:
        self._fail_count += 1
        self._last_error = reason
        self.get_logger().warn(
            f"global setpoint plan rejected: {reason}", throttle_duration_sec=2.0
        )
        self._publish_diagnostics()

    def _note_success(self, diagnostics: dict) -> None:
        self._success_count += 1
        if diagnostics:
            self._publish_diagnostics(extra=diagnostics)

    def _publish_diagnostics(self, extra: dict | None = None) -> None:
        payload = _diagnostics_payload(self._success_count, self._fail_count, self._last_error)
        if extra:
            payload["backend"] = extra
        msg = String()
        msg.data = json.dumps(payload)
        self._diagnostics_pub.publish(msg)


class OnlineMpcPlannerSource(Node):
    """Target stream/cache → online MPC backend step → command sink."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self._configured = False

    def configure(
        self,
        *,
        backend: OnlineMpcBackend,
        joint_names: list[str],
        config: OnlineMpcSourceConfig,
        command_sink: CommandSink | None = None,
        robot_context: RobotContext | None = None,
    ) -> None:
        if self._configured:
            raise RuntimeError(f"{self.get_name()} is already configured")
        if config.step_rate_hz <= 0.0:
            raise ValueError("step_rate_hz must be positive")

        self._backend = backend
        self._state_provider = CachedJointStateProvider(joint_names, config.max_state_age_s)
        self._robot_context = robot_context or RobotContext(
            state_provider=self._state_provider,
            model_provider=StaticRobotModelProvider(
                RobotModelInfo(joint_names=list(joint_names))
            ),
        )
        self._joint_names = list(joint_names)
        self._config = config
        self._backend_initialized = False
        self._last_target: Optional[Target] = None
        self._last_target_stamp_s: Optional[float] = None
        self._success_count = 0
        self._fail_count = 0
        self._last_error: Optional[str] = None

        self._command_sink = command_sink or make_command_sink(
            self,
            mode=config.command_sink_mode,
            source_name=config.source_name,
            source_namespace=config.source_namespace,
            direct_command_topic=config.direct_command_topic,
        )
        self._diagnostics_pub = self.create_publisher(String, "~/diagnostics", 10)

        self.create_subscription(JointState, config.state_topic, self._on_state, 10)

        self._backend.warmup()
        self.create_timer(1.0 / config.step_rate_hz, self._on_step_timer)
        self._configured = True

        self.get_logger().info(
            f"{self.get_name()}: online MPC step_rate={config.step_rate_hz:.1f}Hz "
            f"source={config.source_namespace.rstrip('/')}/{config.source_name}"
        )

    def set_target(self, target: Target, stamp_s: float) -> None:
        self._last_target = target
        self._last_target_stamp_s = stamp_s

    def _on_state(self, msg: JointState) -> None:
        stamp_s = _stamp_or_now_s(self, msg)
        self._state_provider.update(
            list(msg.name),
            list(msg.position),
            list(msg.velocity) if msg.velocity else None,
            stamp_s,
        )

    def _on_step_timer(self) -> None:
        now_s = _now_s(self)
        current_state = self._robot_context.get_current_state(now_s)
        if current_state is None:
            self._note_failure(_state_error_reason(self._state_provider))
            return

        if not self._backend_initialized:
            self._backend.reset(current_state)
            self._backend_initialized = True

        if self._last_target is None or self._last_target_stamp_s is None:
            self._note_failure("target_missing")
            return
        if (now_s - self._last_target_stamp_s) > self._config.max_target_age_s:
            self._note_failure("target_stale")
            return

        dt = 1.0 / self._config.step_rate_hz
        try:
            self._backend.update_target(self._last_target)
            result = self._backend.step(current_state, dt)
        except Exception as exc:  # noqa: BLE001
            self._note_failure(f"backend_exception:{exc}")
            return

        if not result.valid or not result.points:
            self._note_failure(result.reason or "backend_invalid_result")
            return
        if not _horizon_points_are_finite(result.points, len(current_state.joint_names)):
            self._note_failure("non_finite_or_length_mismatch")
            return

        self._command_sink.publish_joint_chunk(
            self, current_state.joint_names, result.points
        )
        self._note_success()

    def _note_failure(self, reason: str) -> None:
        self._fail_count += 1
        self._last_error = reason
        self.get_logger().warn(
            f"online MPC step rejected: {reason}", throttle_duration_sec=2.0
        )
        self._publish_diagnostics()

    def _note_success(self) -> None:
        self._success_count += 1

    def _publish_diagnostics(self) -> None:
        msg = String()
        msg.data = json.dumps(
            _diagnostics_payload(self._success_count, self._fail_count, self._last_error)
        )
        self._diagnostics_pub.publish(msg)


class GlobalTrajectoryPlannerRuntime:
    """Non-node helper for request-oriented global trajectory dispatch."""

    def __init__(self, backend: GlobalTrajectoryBackend, command_sink: CommandSink) -> None:
        self._backend = backend
        self._command_sink = command_sink
        self._backend.warmup()

    def plan(
        self,
        start_state: StartState,
        target: Target,
        options: dict,
        world: World | None = None,
    ) -> TrajectoryPlanResult:
        if world is not None:
            self._backend.update_world(world)
        return self._backend.plan(start_state, target, options)

    def publish(self, node: Node, result: TrajectoryPlanResult) -> None:
        if not result.valid or result.joint_names is None:
            raise ValueError(result.reason or "invalid trajectory plan")
        self._command_sink.publish_joint_trajectory(node, result.joint_names, result.points)


def _stamp_or_now_s(node: Node, msg) -> float:  # noqa: ANN001
    stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return stamp_s if stamp_s > 0.0 else _now_s(node)


def _now_s(node: Node) -> float:
    return node.get_clock().now().nanoseconds / 1e9


def _state_error_reason(state_provider: CachedJointStateProvider) -> str:
    missing = state_provider.last_missing_joints
    return f"state_missing_joints:{missing}" if missing else "state_stale_or_missing"


def _horizon_points_are_finite(points: list[HorizonPlanPoint], expected_len: int) -> bool:
    for point in points:
        if len(point.positions) != expected_len:
            return False
        if not all(math.isfinite(v) for v in point.positions):
            return False
        if point.velocities is not None:
            if len(point.velocities) != expected_len:
                return False
            if not all(math.isfinite(v) for v in point.velocities):
                return False
    return True


def _diagnostics_payload(
    success_count: int,
    fail_count: int,
    last_error: Optional[str],
) -> dict:
    return {
        "success_count": success_count,
        "fail_count": fail_count,
        "last_error": last_error,
    }
