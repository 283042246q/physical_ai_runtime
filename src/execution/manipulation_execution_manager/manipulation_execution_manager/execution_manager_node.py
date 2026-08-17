#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""M2 Execution Manager — multi-source runtime arbitration and joint+pose-space normalization.

M2 extends M1 with DAgger-compatible runtime source arbitration:

  source nodes (teleop / planner / policy / debug)
    → /action_sources/<name>/joint_target  (JointState)
      or /action_sources/<name>/joint_chunk (JointTrajectory)
      or /action_sources/<name>/pose_target   (PoseStamped)
      or /action_sources/<name>/pose_chunk    (PoseArray)
      or /action_sources/<name>/twist_target  (TwistStamped)
    → execution manager (source gate / decoder / normalization / ARBITRATION)
    → /position_controller/joint_reference  (JointTrajectory)
      /position_controller/pose_reference   (PoseStamped)
      /position_controller/pose_chunk_reference  (PoseArray)
      /position_controller/twist_reference  (TwistStamped)
    → JointSpacePositionController / TaskSpaceKinematicPositionController

Arbitration (M2):
  - All configured sources are subscribed eagerly (always ready).
  - Priority-based preemption: higher priority source wins immediately.
  - Implicit takeover: human teleop starts publishing → auto-switch.
  - Automatic fallback: human stops → reverts to next priority source.
  - Anti-flapping hold prevents rapid toggling between same-priority sources
    (joint path; see R-04/EM-106 for pose/twist and cross-route).
  - Per-source observability published in status for DAgger segment annotation.
  - JSPC and TSKPC are mutually exclusive: a single global arbiter picks the
    highest-priority fresh streaming source across joint, pose, and twist; only
    its route publishes, the other is suppressed.
  - joint_trajectory_goal is a parallel, independent path to the JTC controller
    and does not participate in JSPC/TSKPC arbitration.

Deployment model:
  One source publishes one modality (joint OR pose OR twist); arbitration is
  across sources, not across modalities on one source. Primary use case is
  HITL/DAgger: a continuous policy source preempted by a higher-priority human
  teleop source. See docs/architecture.md for details.

Backward compatibility:
  If the ``sources`` dict is absent but ``active_source`` is set (M1 config),
  a single virtual source with priority 0 is created.  M1 launch files work
  unchanged.  Pose subscriptions are opt-in: sources with `pose_contracts: []`
  or without the key skip pose wiring.

See docs/architecture.md and docs/contracts.md for the EM contract reference.
"""

from __future__ import annotations

import json
import math
import sys as _sys
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from . import pose_utils
from .execution_manager_core import (
    DEFAULT_JOINT_NAMES,
    DEFAULT_MIN_HOLD_DURATION_S,
    CONTRACT_JOINT_TARGET,
    CONTRACT_JOINT_CHUNK,
    CONTRACT_JOINT_TRAJECTORY_GOAL,
    CONTRACT_POSE_TARGET,
    CONTRACT_POSE_CHUNK,
    CONTRACT_DELTA_POSE,
    CONTRACT_DELTA_CHUNK,
    CONTRACT_TWIST_TARGET,
    ROUTE_JSPC,
    ROUTE_TSKPC,
    STATE_IDLE,
    STATE_ARMED,
    STATE_ACTIVE,
    STATE_DEGRADED,
    SourceState,
    decode_joint_target,
    decode_joint_chunk,
    decode_pose_target,
    decode_pose_chunk,
    decode_delta_pose,
    decode_delta_chunk,
)
from .execution_manager_arbitration import (
    select_active_streaming_source,
    select_winning_source,
    select_winning_pose_source,
    select_winning_twist_source,
    should_switch_to,
)
from .execution_manager_status import build_status_snapshot
from .execution_manager_validation import (
    STAMP_FUTURE_REJECTED,
    STAMP_STALE_REJECTED,
    STAMP_ZERO_REJECTED,
    validate_reference_stamp,
)
from .execution_manager_ingest import (
    apply_valid_joint_reference,
    apply_valid_pose_reference,
    apply_valid_twist_reference,
    decode_joint_reference,
    decode_pose_reference,
    decode_twist_reference,
    joint_reference_drop_reason,
)


# ---------------------------------------------------------------------------
# QoS profiles (see docs/contracts.md)
# ---------------------------------------------------------------------------

def _reference_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )


def _status_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


def parse_sources_config(value: object) -> Dict[str, dict]:
    """Parse and validate the public ``sources`` JSON-object parameter."""
    if not isinstance(value, str):
        raise RuntimeError("sources must be a JSON object encoded as a string")
    if not value.strip():
        return {}
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"sources is not valid JSON: {exc.msg}") from exc
    if not isinstance(parsed, dict):
        raise RuntimeError("sources JSON must decode to an object keyed by source name")

    for source_name, config in parsed.items():
        if not isinstance(source_name, str) or not source_name.strip():
            raise RuntimeError("sources keys must be non-empty strings")
        if not isinstance(config, dict):
            raise RuntimeError(f"sources.{source_name} must be a JSON object")
        priority = config.get("priority", 0)
        if isinstance(priority, bool) or not isinstance(priority, int):
            raise RuntimeError(f"sources.{source_name}.priority must be an integer")
        inactive_timeout = config.get("inactive_timeout_s")
        if inactive_timeout is not None and (
            isinstance(inactive_timeout, bool)
            or not isinstance(inactive_timeout, (int, float))
            or not math.isfinite(float(inactive_timeout))
            or float(inactive_timeout) <= 0.0
        ):
            raise RuntimeError(
                f"sources.{source_name}.inactive_timeout_s must be finite and positive"
            )
        for key in ("pose_contracts", "twist_contracts", "goal_contracts"):
            contracts = config.get(key, [])
            if not isinstance(contracts, (str, list)) or (
                isinstance(contracts, list)
                and not all(isinstance(item, str) for item in contracts)
            ):
                raise RuntimeError(
                    f"sources.{source_name}.{key} must be a string or string array"
                )
    return parsed


# ---------------------------------------------------------------------------
# Execution Manager Node (M2: multi-source arbitration)
# ---------------------------------------------------------------------------

class ExecutionManager(Node):
    """M2 execution manager: runtime multi-source arbitration for DAgger.

    State machine (global):
      IDLE      — no sources configured or no valid message yet.
      ARMED     — at least one source has produced a valid message.
      ACTIVE    — publishing from the winning source (arbitrator-selected).
      DEGRADED  — all sources inactive/stale; publishing stopped.

    Within ACTIVE, the winning_source changes dynamically via the arbitrator
    without leaving the ACTIVE state. Only when NO source is viable does the
    state transition to DEGRADED.

    FAULT is intentionally absent from the runtime state machine.
    """

    def __init__(self) -> None:
        super().__init__("execution_manager")

        # --- Parameters ---
        joint_names_param = (
            self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES).value
        )
        self._joint_names = self._parse_joint_names(joint_names_param)

        # M1 backward-compat: active_source (used only if sources dict is empty)
        self._legacy_active_source = str(
            self.declare_parameter("active_source", "none").value
        )
        self._source_namespace = str(
            self.declare_parameter("source_namespace", "/action_sources").value
        ).rstrip("/")
        self._output_topic = str(
            self.declare_parameter(
                "output_topic", "/position_controller/joint_reference"
            ).value
        )
        self._pose_output_topic = str(
            self.declare_parameter(
                "pose_output_topic", "/position_controller/pose_reference"
            ).value
        )
        self._pose_chunk_output_topic = str(
            self.declare_parameter(
                "pose_chunk_output_topic", "/position_controller/pose_chunk_reference"
            ).value
        )
        self._twist_output_topic = str(
            self.declare_parameter(
                "twist_output_topic", "/position_controller/twist_reference"
            ).value
        )
        self._status_topic = str(
            self.declare_parameter(
                "status_topic", "/execution_manager/status"
            ).value
        )
        self._status_rate_hz = float(
            self.declare_parameter("status_rate_hz", 2.0).value
        )
        self._stale_timeout_s = float(
            self.declare_parameter("stale_timeout_s", 0.5).value
        )
        self._max_future_s = float(
            self.declare_parameter("max_future_s", 0.1).value
        )
        self._reject_zero_stamped = bool(
            self.declare_parameter("reject_zero_stamped_references", False).value
        )
        self._arm_to_active_count = int(
            self.declare_parameter("arm_to_active_count", 1).value
        )
        self._degraded_to_active_count = int(
            self.declare_parameter("degraded_to_active_count", 3).value
        )
        self._joint_limits_lower = self._parse_float_list(
            self.declare_parameter("joint_limits_lower", "").value
        )
        self._joint_limits_upper = self._parse_float_list(
            self.declare_parameter("joint_limits_upper", "").value
        )

        # M2: multi-source config (NEW)
        # If absent/empty and active_source != "none", a single virtual source
        # with priority 0 provides backward compatibility.
        # ROS 2 parameters do not support arbitrary nested dictionaries. Keep
        # this public contract unambiguous: one JSON object encoded as a string.
        sources_raw = self.declare_parameter("sources", "").value
        self._sources_config = parse_sources_config(sources_raw)

        # M2: anti-flapping hold (configurable, default 50 ms)
        self._min_hold_duration_s = float(
            self.declare_parameter("min_hold_duration_s", DEFAULT_MIN_HOLD_DURATION_S).value
        )
        self._jtc_action_name = str(
            self.declare_parameter(
                "jtc_action_name", "/arm_controller/follow_joint_trajectory"
            ).value
        )

        # --- Validate parameters ---
        if not self._joint_names:
            raise RuntimeError("joint_names must not be empty")
        if self._status_rate_hz <= 0.0 or not math.isfinite(self._status_rate_hz):
            raise RuntimeError("status_rate_hz must be finite and positive")
        if self._stale_timeout_s <= 0.0:
            raise RuntimeError("stale_timeout_s must be positive")
        if self._max_future_s < 0.0:
            raise RuntimeError("max_future_s must be non-negative")
        if self._arm_to_active_count < 1:
            raise RuntimeError("arm_to_active_count must be >= 1")
        if self._degraded_to_active_count < 1:
            raise RuntimeError("degraded_to_active_count must be >= 1")
        if self._min_hold_duration_s < 0.0:
            raise RuntimeError("min_hold_duration_s must be non-negative")

        # --- State machine ---
        self._state = STATE_IDLE
        self._global_consecutive_fresh_count = 0  # for DEGRADED→ACTIVE recovery

        # --- M2: source registry ---
        self._sources: Dict[str, SourceState] = {}
        self._winning_source: Optional[str] = None
        self._hold_until: Optional[rclpy.time.Time] = None

        # --- Publishers ---
        self._ref_pub = self.create_publisher(
            JointTrajectory, self._output_topic, _reference_qos()
        )
        self._pose_ref_pub = self.create_publisher(
            PoseStamped, self._pose_output_topic, _reference_qos()
        )
        self._pose_chunk_ref_pub = self.create_publisher(
            PoseArray, self._pose_chunk_output_topic, _reference_qos()
        )
        self._twist_ref_pub = self.create_publisher(
            TwistStamped, self._twist_output_topic, _reference_qos()
        )
        self._status_pub = self.create_publisher(
            String, self._status_topic, _status_qos()
        )

        # JTC action client (for joint_trajectory_goal → FollowJointTrajectory)
        self._jtc_client = ActionClient(self, FollowJointTrajectory,
                                         self._jtc_action_name)

        # --- Setup ---
        self._setup_sources()

        # Timers
        freshness_check_hz = max(10.0, self._status_rate_hz * 2.0)
        self._status_timer = self.create_timer(
            1.0 / self._status_rate_hz, self._publish_status
        )
        self._freshness_timer = self.create_timer(
            1.0 / freshness_check_hz, self._check_freshness
        )

        # Startup log
        source_names = list(self._sources.keys()) if self._sources else []
        self.get_logger().info(
            "Execution Manager M2 initialized: sources=%s state=%s output=%s "
            "stale_timeout=%.2fs max_future=%.2fs status_rate=%.1fHz joints=%d "
            "min_hold=%.3fs"
            % (
                source_names if source_names else (
                    [self._legacy_active_source]
                    if self._legacy_active_source != "none"
                    else ["none"]
                ),
                self._state,
                self._output_topic,
                self._stale_timeout_s,
                self._max_future_s,
                self._status_rate_hz,
                len(self._joint_names),
                self._min_hold_duration_s,
            )
        )

    # -----------------------------------------------------------------------
    # Source setup (M2: multi-source with backward compat)
    # -----------------------------------------------------------------------

    def _setup_sources(self) -> None:
        """Create subscriptions for all configured sources.

        If ``sources`` dict is populated, subscribe to every entry at
        ``/action_sources/<name>/joint_target`` and ``joint_chunk``.

        If ``sources`` is empty but ``active_source`` is set to a non-"none"
        value (M1 config), create a single virtual source with priority 0 for
        backward compatibility.

        If ``active_source`` is "none" and ``sources`` is empty, run in
        status-only (IDLE) mode.
        """
        # Resolve the effective source list
        effective: Dict[str, dict] = {}
        if self._sources_config:
            effective = dict(self._sources_config)
        elif self._legacy_active_source and self._legacy_active_source != "none":
            # M1 backward compat: single virtual source
            effective = {
                self._legacy_active_source: {
                    "priority": 0,
                    "inactive_timeout_s": self._stale_timeout_s,
                }
            }
            self.get_logger().info(
                "M1 backward-compat mode: single source '%s' with priority 0"
                % self._legacy_active_source
            )

        if not effective:
            self.get_logger().info(
                "Execution manager: no sources configured; status only, "
                "no motion command subscriptions."
            )
            return

        # Create source entries and subscriptions
        for source_name, config in effective.items():
            priority = int(config.get("priority", 0))
            inactive_timeout = float(
                config.get("inactive_timeout_s", self._stale_timeout_s)
            )

            src = SourceState(
                name=source_name,
                priority=priority,
                inactive_timeout_s=inactive_timeout,
            )
            self._sources[source_name] = src

            target_topic = (
                f"{self._source_namespace}/{source_name}/joint_target"
            )
            chunk_topic = (
                f"{self._source_namespace}/{source_name}/joint_chunk"
            )

            src.target_sub = self.create_subscription(
                JointState,
                target_topic,
                self._make_joint_target_cb(source_name),
                _reference_qos(),
            )
            src.chunk_sub = self.create_subscription(
                JointTrajectory,
                chunk_topic,
                self._make_joint_chunk_cb(source_name),
                _reference_qos(),
            )

            # Pose subscriptions — opt-in per source: check `pose_contracts` key.
            pose_contracts = config.get("pose_contracts", [])
            if isinstance(pose_contracts, str):
                pose_contracts = [v.strip() for v in pose_contracts.split(",")]
            if pose_contracts:
                pose_target_topic = (
                    f"{self._source_namespace}/{source_name}/pose_target"
                )
                pose_chunk_topic = (
                    f"{self._source_namespace}/{source_name}/pose_chunk"
                )
                src.pose_target_sub = self.create_subscription(
                    PoseStamped,
                    pose_target_topic,
                    self._make_pose_target_cb(source_name),
                    _reference_qos(),
                )
                src.pose_chunk_sub = self.create_subscription(
                    PoseArray,
                    pose_chunk_topic,
                    self._make_pose_chunk_cb(source_name),
                    _reference_qos(),
                )
                self.get_logger().info(
                    "Source '%s': pose contracts=%s target=%s chunk=%s"
                    % (source_name, pose_contracts,
                       pose_target_topic, pose_chunk_topic)
                )

            # Twist subscriptions — opt-in per source.
            twist_contracts = config.get("twist_contracts", [])
            if isinstance(twist_contracts, str):
                twist_contracts = [v.strip() for v in twist_contracts.split(",")]
            if "twist_target" in twist_contracts:
                twist_target_topic = (
                    f"{self._source_namespace}/{source_name}/twist_target"
                )
                src.twist_target_sub = self.create_subscription(
                    TwistStamped,
                    twist_target_topic,
                    self._make_twist_target_cb(source_name),
                    _reference_qos(),
                )
                self.get_logger().info(
                    "Source '%s': twist_target topic=%s"
                    % (source_name, twist_target_topic)
                )

            # Goal subscriptions — opt-in per source: check `goal_contracts` key.
            goal_contracts = config.get("goal_contracts", [])
            if isinstance(goal_contracts, str):
                goal_contracts = [v.strip() for v in goal_contracts.split(",")]
            if "joint_trajectory_goal" in goal_contracts:
                goal_topic = (
                    f"{self._source_namespace}/{source_name}/joint_trajectory_goal"
                )
                src.joint_goal_sub = self.create_subscription(
                    JointTrajectory,
                    goal_topic,
                    self._make_joint_goal_cb(source_name),
                    _reference_qos(),
                )
                self.get_logger().info(
                    "Source '%s': joint_goal topic=%s → JTC action %s"
                    % (source_name, goal_topic, self._jtc_action_name)
                )

            self.get_logger().info(
                "Source '%s': priority=%d inactive_timeout=%.3fs target=%s chunk=%s"
                % (source_name, priority, inactive_timeout, target_topic, chunk_topic)
            )

    # -----------------------------------------------------------------------
    # Callback factories (M2: per-source closures avoid late-binding issues)
    # -----------------------------------------------------------------------

    def _make_joint_target_cb(self, source_name: str):
        """Return a callback that captures source_name for JointState messages."""
        def cb(msg: JointState) -> None:
            self._process_incoming(source_name, msg, msg.header.stamp,
                                   CONTRACT_JOINT_TARGET)
        return cb

    def _make_joint_chunk_cb(self, source_name: str):
        """Return a callback that captures source_name for JointTrajectory messages."""
        def cb(msg: JointTrajectory) -> None:
            self._process_incoming(source_name, msg, msg.header.stamp,
                                   CONTRACT_JOINT_CHUNK)
        return cb

    def _make_pose_target_cb(self, source_name: str):
        """Return a callback that captures source_name for PoseStamped messages."""
        def cb(msg: PoseStamped) -> None:
            self._process_pose_incoming(source_name, msg, msg.header.stamp,
                                        CONTRACT_POSE_TARGET)
        return cb

    def _make_pose_chunk_cb(self, source_name: str):
        """Return a callback that captures source_name for PoseArray messages."""
        def cb(msg: PoseArray) -> None:
            self._process_pose_incoming(source_name, msg, msg.header.stamp,
                                        CONTRACT_POSE_CHUNK)
        return cb

    def _make_twist_target_cb(self, source_name: str):
        """Return a callback that captures source_name for TwistStamped messages."""
        def cb(msg: TwistStamped) -> None:
            self._process_twist_incoming(source_name, msg, msg.header.stamp,
                                         CONTRACT_TWIST_TARGET)
        return cb

    def _make_joint_goal_cb(self, source_name: str):
        """Return a callback that captures source_name for joint_trajectory_goal."""
        def cb(msg: JointTrajectory) -> None:
            self._process_incoming(source_name, msg, msg.header.stamp,
                                   CONTRACT_JOINT_TRAJECTORY_GOAL)
        return cb

    # -----------------------------------------------------------------------
    # Message processing (M2: per-source routing)
    # -----------------------------------------------------------------------

    def _process_incoming(
        self, source_name: str, msg, stamp, contract_type: str
    ) -> None:
        """Process an incoming message from a specific source.

        Time validity rules (per design §9 + handoff §6 risk 1):
          - Zero stamp: fallback to now() or reject (configurable).
          - Future stamp beyond max_future_s: REJECT.
          - Past stamp beyond stale_timeout_s: REJECT.
        """
        source = self._sources.get(source_name)
        if source is None:
            return

        now = self.get_clock().now()

        # --- Time validity check ---
        stamp_result = validate_reference_stamp(
            stamp=stamp,
            now=now,
            reject_zero_stamp=self._reject_zero_stamped,
            max_future_s=self._max_future_s,
            stale_timeout_s=self._stale_timeout_s,
        )
        if not stamp_result.valid:
            if stamp_result.code == STAMP_ZERO_REJECTED:
                source.invalid_stamp_count += 1
                source.last_error = "zero-stamped reference rejected"
            elif stamp_result.code == STAMP_FUTURE_REJECTED:
                source.invalid_stamp_count += 1
                source.last_error = (
                    f"future stamp rejected: {stamp_result.latency_ms:.1f} ms ahead"
                )
            elif stamp_result.code == STAMP_STALE_REJECTED:
                source.stale_count += 1
                source.last_error = (
                    f"stale stamp rejected: {stamp_result.latency_ms:.1f} ms old"
                )
            self.get_logger().warn(
                "[%s] %s" % (source_name, source.last_error),
                throttle_duration_sec=2.0,
            )
            return

        # --- Decode and normalize ---
        normalized = decode_joint_reference(
            contract_type=contract_type,
            msg=msg,
            joint_names=self._joint_names,
            joint_limits_lower=self._joint_limits_lower,
            joint_limits_upper=self._joint_limits_upper,
            clock=self.get_clock(),
            reject_zero_stamp=self._reject_zero_stamped,
        )
        drop_reason = joint_reference_drop_reason(normalized)
        if drop_reason == "decode failed":
            source.decode_fail_count += 1
            source.last_error = f"decode failed for contract={contract_type}"
            self.get_logger().warn(
                "[%s] %s" % (source_name, source.last_error),
                throttle_duration_sec=2.0,
            )
            return
        if drop_reason is not None:
            source.normalize_drop_count += 1
            source.last_error = drop_reason
            self.get_logger().warn(
                "[%s] %s" % (source_name, source.last_error),
                throttle_duration_sec=2.0,
            )
            return

        # --- JTC goal: parallel, independent path ---
        # joint_trajectory_goal targets a separate action-based controller and
        # must NOT populate the JSPC streaming buffer (latest_reference) or the
        # streaming route, so it never participates in JSPC/TSKPC arbitration.
        if contract_type == CONTRACT_JOINT_TRAJECTORY_GOAL:
            source.last_received_stamp = now
            source.consecutive_valid_count += 1
            if stamp_result.latency_ms is not None:
                source.last_normalize_latency_ms = max(0.0, stamp_result.latency_ms)
            self._send_jtc_goal(normalized)
            return

        # --- Store decoded reference in source state (JSPC streaming) ---
        apply_valid_joint_reference(
            source=source,
            normalized=normalized,
            contract_type=contract_type,
            now=now,
            latency_ms=stamp_result.latency_ms,
        )
        source.latest_stream_route = ROUTE_JSPC

        # --- Global state machine transitions ---
        if self._state == STATE_IDLE:
            self._transition_to(STATE_ARMED)
        elif self._state == STATE_ARMED:
            winner = self._select_winning_source()
            if winner is not None:
                winner_src = self._sources[winner]
                if winner_src.consecutive_valid_count >= self._arm_to_active_count:
                    self._winning_source = winner
                    self._transition_to(STATE_ACTIVE)
        elif self._state == STATE_DEGRADED:
            # DEGRADED→ACTIVE handled by freshness timer
            pass

        if self._state == STATE_ACTIVE:
            self._publish_from_arbitrator()

    # -----------------------------------------------------------------------
    # M2: Priority arbitration
    # -----------------------------------------------------------------------

    def _select_winning_source(self) -> Optional[str]:
        """Select the highest-priority fresh source."""
        return select_winning_source(
            self._sources,
            self.get_clock().now(),
            self._stale_timeout_s,
        )

    def _should_switch_to(self, candidate_name: Optional[str]) -> bool:
        """Decide whether to switch the output to *candidate_name*."""
        return should_switch_to(
            candidate_name,
            self._winning_source,
            self._sources,
            self._hold_until,
            self.get_clock().now(),
        )

    def _active_streaming_route(self) -> Optional[str]:
        """Return the currently active streaming route (JSPC/TSKPC), or None.

        Enforces JSPC/TSKPC mutual exclusion: the single highest-priority fresh
        streaming source across all routes decides which route may publish.
        """
        winner = select_active_streaming_source(
            self._sources, self.get_clock().now(), self._stale_timeout_s
        )
        if winner is None:
            return None
        return self._sources[winner].latest_stream_route

    # -----------------------------------------------------------------------
    # Publishing (M2: arbitrator-gated)
    # -----------------------------------------------------------------------

    def _publish_from_arbitrator(self) -> None:
        """Select the winning source and publish its latest reference.

        Called after every incoming message and periodically from the freshness
        monitor. Only the winning source's reference is published, and only when
        JSPC is the active route (TSKPC is suppressed for mutual exclusion).
        """
        if self._active_streaming_route() != ROUTE_JSPC:
            return
        winner = self._select_winning_source()
        if winner is None:
            return

        if not self._should_switch_to(winner):
            return

        # Record source switch
        if winner != self._winning_source:
            old = self._winning_source
            self._winning_source = winner
            self._hold_until = self.get_clock().now() + rclpy.duration.Duration(
                seconds=self._min_hold_duration_s
            )
            self.get_logger().info(
                "Arbitrator: source switch %s -> %s (priority %d)"
                % (
                    old if old else "(none)",
                    winner,
                    self._sources[winner].priority,
                )
            )

        src = self._sources[winner]
        if src.latest_reference is not None:
            self._ref_pub.publish(src.latest_reference)

    # -----------------------------------------------------------------------
    # JTC action goal routing
    # -----------------------------------------------------------------------

    def _send_jtc_goal(self, trajectory: JointTrajectory) -> None:
        """Send a decoded JointTrajectory to the JTC action server.

        No state-machine gate — goal contracts bypass the streaming arbitrator.
        The JTC action server owns goal lifecycle (accept → active → succeed/abort).

        The server-readiness check is non-blocking: the EM is event-driven and
        must never stall the executor waiting on a downstream server.
        """
        if not self._jtc_client.server_is_ready():
            self.get_logger().warn(
                "JTC action server '%s' not available; dropping goal.",
                self._jtc_action_name,
                throttle_duration_sec=5.0,
            )
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self._jtc_client.send_goal_async(goal_msg)
        self.get_logger().debug(
            "JTC goal sent: %d points, %d joints"
            % (len(trajectory.points), len(trajectory.joint_names))
        )

    # -----------------------------------------------------------------------
    # Pose-space processing (M2: passthrough validation + delta→absolute)
    # -----------------------------------------------------------------------

    def _process_pose_incoming(
        self, source_name: str, msg, stamp, contract_type: str
    ) -> None:
        """Process an incoming pose message: validate, normalize, publish.

        The pose path does not participate in the full M2 state machine;
        it publishes the most recent valid pose from the active pose source
        immediately after decoding.  Delta contracts accumulate a reference
        pose inside SourceState.ref_pose.
        """
        source = self._sources.get(source_name)
        if source is None:
            return

        now = self.get_clock().now()

        # --- Time validity check (same rules as joint path) ---
        stamp_result = validate_reference_stamp(
            stamp=stamp,
            now=now,
            reject_zero_stamp=self._reject_zero_stamped,
            max_future_s=self._max_future_s,
            stale_timeout_s=self._stale_timeout_s,
        )
        if not stamp_result.valid:
            if stamp_result.code == STAMP_ZERO_REJECTED:
                source.invalid_stamp_count += 1
                source.last_error = "zero-stamped pose rejected"
            elif stamp_result.code == STAMP_FUTURE_REJECTED:
                source.invalid_stamp_count += 1
                source.last_error = (
                    f"future pose stamp rejected: {stamp_result.latency_ms:.1f} ms ahead"
                )
            elif stamp_result.code == STAMP_STALE_REJECTED:
                source.stale_count += 1
                source.last_error = (
                    f"stale pose stamp rejected: {stamp_result.latency_ms:.1f} ms old"
                )
            return

        # --- Decode ---
        decoded, is_chunk, ref_pose_update = decode_pose_reference(
            contract_type=contract_type,
            msg=msg,
            source_ref_pose=source.ref_pose,
        )

        if decoded is None:
            source.decode_fail_count += 1
            source.last_error = f"pose decode failed for {contract_type}"
            return

        apply_valid_pose_reference(
            source=source,
            decoded=decoded,
            is_chunk=is_chunk,
            now=now,
            ref_pose_update=ref_pose_update,
            latency_ms=stamp_result.latency_ms,
        )
        source.latest_stream_route = ROUTE_TSKPC

        # --- Publish pose output ---
        self._publish_pose_output()

    def _publish_pose_output(self) -> None:
        """Select the winning pose source and publish its latest decoded pose.

        Pose arbitration: highest-priority source with a fresh pose/chunk wins,
        using the same eligibility policy (per-source inactivity + global stale)
        as the joint and twist paths.  Chunk is preferred over single target
        when both are available from the same source. Suppressed when JSPC is
        the active route (JSPC/TSKPC mutual exclusion).
        """
        if self._active_streaming_route() != ROUTE_TSKPC:
            return
        now = self.get_clock().now()
        winner = select_winning_pose_source(
            self._sources, now, self._stale_timeout_s
        )
        if winner is None:
            return
        best = self._sources[winner]

        # Prefer chunk over single target from the same source.
        if best.latest_pose_chunk is not None:
            self._pose_chunk_ref_pub.publish(best.latest_pose_chunk)
        elif best.latest_pose_target is not None:
            self._pose_ref_pub.publish(best.latest_pose_target)

    # -----------------------------------------------------------------------
    # Twist-space processing (passthrough validation)
    # -----------------------------------------------------------------------

    def _process_twist_incoming(
        self, source_name: str, msg: TwistStamped, stamp, contract_type: str
    ) -> None:
        """Process an incoming TwistStamped: validate, normalize, publish.

        EM does not filter, integrate, or transform twist commands. Source
        nodes own device semantics, deadband/gains, and frame alignment.
        """
        source = self._sources.get(source_name)
        if source is None:
            return

        now = self.get_clock().now()

        stamp_result = validate_reference_stamp(
            stamp=stamp,
            now=now,
            reject_zero_stamp=self._reject_zero_stamped,
            max_future_s=self._max_future_s,
            stale_timeout_s=self._stale_timeout_s,
        )
        if not stamp_result.valid:
            if stamp_result.code == STAMP_ZERO_REJECTED:
                source.invalid_stamp_count += 1
                source.last_error = "zero-stamped twist rejected"
            elif stamp_result.code == STAMP_FUTURE_REJECTED:
                source.invalid_stamp_count += 1
                source.last_error = (
                    f"future twist stamp rejected: {stamp_result.latency_ms:.1f} ms ahead"
                )
            elif stamp_result.code == STAMP_STALE_REJECTED:
                source.stale_count += 1
                source.last_error = (
                    f"stale twist stamp rejected: {stamp_result.latency_ms:.1f} ms old"
                )
            return

        decoded = decode_twist_reference(
            contract_type=contract_type,
            msg=msg,
        )
        if decoded is None:
            source.decode_fail_count += 1
            source.last_error = f"twist decode failed for {contract_type}"
            return

        apply_valid_twist_reference(
            source=source,
            decoded=decoded,
            now=now,
            latency_ms=stamp_result.latency_ms,
        )
        source.latest_stream_route = ROUTE_TSKPC

        self._publish_twist_output()

    def _publish_twist_output(self) -> None:
        """Select the winning twist source and publish its latest command.

        Suppressed when JSPC is the active route (JSPC/TSKPC mutual exclusion).
        """
        if self._active_streaming_route() != ROUTE_TSKPC:
            return
        now = self.get_clock().now()
        winner = select_winning_twist_source(
            self._sources, now, self._stale_timeout_s
        )
        if winner is None:
            return
        self._twist_ref_pub.publish(self._sources[winner].latest_twist_target)

    # -----------------------------------------------------------------------
    # Freshness monitor (M2: multi-source)
    # -----------------------------------------------------------------------

    def _check_freshness(self) -> None:
        """Periodic check for source staleness and fallback.

        In ACTIVE: if the winning source goes inactive/stale, try to find a
        new winner before degrading. If no candidate, transition to DEGRADED.

        In DEGRADED: if any source becomes eligible again, count consecutive
        fresh ticks and transition back to ACTIVE when the threshold is met.
        """
        if self._state == STATE_ACTIVE:
            winner = self._select_winning_source()
            if winner is None:
                self._winning_source = None
                self._transition_to(STATE_DEGRADED)
            elif winner != self._winning_source:
                # Fallback: current winner lost, new winner found
                self._publish_from_arbitrator()

        elif self._state == STATE_DEGRADED:
            winner = self._select_winning_source()
            if winner is not None:
                self._global_consecutive_fresh_count += 1
                if (
                    self._global_consecutive_fresh_count
                    >= self._degraded_to_active_count
                ):
                    self._winning_source = winner
                    self._transition_to(STATE_ACTIVE)
                    self._publish_from_arbitrator()
            else:
                self._global_consecutive_fresh_count = 0

    # -----------------------------------------------------------------------
    # State machine transitions
    # -----------------------------------------------------------------------

    def _transition_to(self, new_state: str) -> None:
        """Execute a global state transition with side effects."""
        old_state = self._state
        if old_state == new_state:
            return

        self._state = new_state

        if new_state == STATE_ARMED:
            self._global_consecutive_fresh_count = 0
            for src in self._sources.values():
                src.last_error = ""
        elif new_state == STATE_ACTIVE:
            self._global_consecutive_fresh_count = 0
            for src in self._sources.values():
                src.last_error = ""
                src.consecutive_valid_count = 0
        elif new_state == STATE_DEGRADED:
            self._global_consecutive_fresh_count = 0
            for src in self._sources.values():
                src.consecutive_valid_count = 0

        self.get_logger().info(
            "State transition: %s -> %s" % (old_state, new_state)
        )

    # -----------------------------------------------------------------------
    # Status publishing (M2: per-source observability for DAgger annotation)
    # -----------------------------------------------------------------------

    def _publish_status(self) -> None:
        """Publish execution manager status as JSON."""
        streaming_winner = select_active_streaming_source(
            self._sources, self.get_clock().now(), self._stale_timeout_s
        )
        active_route = (
            self._sources[streaming_winner].latest_stream_route
            if streaming_winner is not None
            else None
        )
        status = build_status_snapshot(
            sources=self._sources,
            now=self.get_clock().now(),
            state=self._state,
            winning_source=self._winning_source,
            stale_timeout_s=self._stale_timeout_s,
            joint_names=self._joint_names,
            consecutive_fresh_count=self._global_consecutive_fresh_count,
            streaming_winner=streaming_winner,
            active_streaming_route=active_route,
        )
        self._status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    # -----------------------------------------------------------------------
    # Static helpers
    # -----------------------------------------------------------------------

    @staticmethod
    def _parse_joint_names(value) -> list[str]:
        if isinstance(value, str):
            return [name.strip() for name in value.split(",") if name.strip()]
        return [str(name) for name in value]

    @staticmethod
    def _parse_float_list(value) -> list[float]:
        """Parse a comma-separated string of floats, or a list of floats."""
        if isinstance(value, str):
            value = value.strip()
            if not value:
                return []
            return [float(v.strip()) for v in value.split(",") if v.strip()]
        if isinstance(value, list):
            return [float(v) for v in value]
        return []


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    _exit_code = 0
    try:
        node = ExecutionManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # Parameter/validation failures are hard errors: surface a non-zero
        # exit so launch systems and CI detect the misconfiguration.
        print(f"execution_manager_node failed: {exc}", file=_sys.stderr)
        _exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    _sys.exit(_exit_code)


if __name__ == "__main__":
    main()
