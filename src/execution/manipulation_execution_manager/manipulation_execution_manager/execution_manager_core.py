#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Pure data structures and decoders for the execution manager.

This module intentionally contains no ROS node, publishers, subscriptions, or
timers.  Keeping these helpers separate makes the EM contract testable without
starting a full rclpy node.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import pose_utils


DEFAULT_JOINT_NAMES = "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6"  # Replace for your robot

# State machine states (see docs/architecture.md).
STATE_IDLE = "IDLE"
STATE_ARMED = "ARMED"
STATE_ACTIVE = "ACTIVE"
STATE_DEGRADED = "DEGRADED"

# Contract types (determine decoder selection)
CONTRACT_JOINT_TARGET = "joint_target"
CONTRACT_JOINT_CHUNK = "joint_chunk"
CONTRACT_JOINT_TRAJECTORY_GOAL = "joint_trajectory_goal"
CONTRACT_POSE_TARGET = "pose_target"
CONTRACT_POSE_CHUNK = "pose_chunk"
CONTRACT_DELTA_POSE = "delta_pose"
CONTRACT_DELTA_CHUNK = "delta_chunk"
CONTRACT_TWIST_TARGET = "twist_target"

POSE_CONTRACTS = {
    CONTRACT_POSE_TARGET,
    CONTRACT_POSE_CHUNK,
    CONTRACT_DELTA_POSE,
    CONTRACT_DELTA_CHUNK,
}

# Contract routing.
#
# Two granularities, intentionally:
#   - SourceState.latest_stream_route uses the COARSE controller family
#     (ROUTE_JSPC vs ROUTE_TSKPC) because JSPC/TSKPC mutual exclusion is what
#     matters for arbitration. Pose and twist both map to ROUTE_TSKPC.
#   - CONTRACT_ROUTE below uses the FINER per-contract label (ROUTE_TSKPC_TWIST
#     distinguishes twist) for documentation/dispatch detail only.
ROUTE_JSPC = "jspc"
ROUTE_JTC = "jtc"
ROUTE_TSKPC = "tskpc"
ROUTE_TSKPC_TWIST = "tskpc_twist"

CONTRACT_ROUTE = {
    CONTRACT_JOINT_TARGET: ROUTE_JSPC,
    CONTRACT_JOINT_CHUNK: ROUTE_JSPC,
    CONTRACT_JOINT_TRAJECTORY_GOAL: ROUTE_JTC,
    CONTRACT_POSE_TARGET: ROUTE_TSKPC,
    CONTRACT_POSE_CHUNK: ROUTE_TSKPC,
    CONTRACT_DELTA_POSE: ROUTE_TSKPC,
    CONTRACT_DELTA_CHUNK: ROUTE_TSKPC,
    CONTRACT_TWIST_TARGET: ROUTE_TSKPC_TWIST,
}

DEFAULT_MIN_HOLD_DURATION_S = 0.05
DEFAULT_POSE_CHUNK_SIZE = 5
DEFAULT_POSE_CHUNK_OVERLAP = 2


def decode_joint_target(
    msg: JointState,
    joint_names: list[str],
    joint_limits_lower: list[float],
    joint_limits_upper: list[float],
    reject_out_of_bounds: bool,
    clock: rclpy.clock.Clock,
    reject_zero_stamp: bool,
) -> Optional[JointTrajectory]:
    """Convert a JointState target into a single-point JointTrajectory."""
    positions_by_name = dict(zip(msg.name, msg.position))
    missing = [name for name in joint_names if name not in positions_by_name]
    if missing:
        return None

    positions = [float(positions_by_name[name]) for name in joint_names]
    if not all(math.isfinite(p) for p in positions):
        return None

    for i, pos in enumerate(positions):
        lower = joint_limits_lower[i] if i < len(joint_limits_lower) else -math.inf
        upper = joint_limits_upper[i] if i < len(joint_limits_upper) else math.inf
        if math.isfinite(lower) and pos < lower:
            if reject_out_of_bounds:
                return None
            positions[i] = lower
        if math.isfinite(upper) and pos > upper:
            if reject_out_of_bounds:
                return None
            positions[i] = upper

    stamp = msg.header.stamp
    if stamp.sec == 0 and stamp.nanosec == 0:
        if reject_zero_stamp:
            return None
        stamp = clock.now().to_msg()

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = 0

    out = JointTrajectory()
    out.header.stamp = stamp
    out.header.frame_id = msg.header.frame_id
    out.joint_names = list(joint_names)
    out.points = [point]
    return out


def decode_joint_chunk(
    msg: JointTrajectory,
    joint_names: list[str],
    joint_limits_lower: list[float],
    joint_limits_upper: list[float],
    reject_out_of_bounds: bool,
) -> Optional[JointTrajectory]:
    """Reorder and validate a JointTrajectory chunk."""
    if not msg.joint_names or not msg.points:
        return None

    index_by_name = {name: idx for idx, name in enumerate(msg.joint_names)}
    missing = [name for name in joint_names if name not in index_by_name]
    if missing:
        return None

    out = JointTrajectory()
    out.header = msg.header
    out.joint_names = list(joint_names)

    for point in msg.points:
        if len(point.positions) < len(msg.joint_names):
            return None

        normalized = JointTrajectoryPoint()
        reordered = [float(point.positions[index_by_name[name]]) for name in joint_names]
        if not all(math.isfinite(p) for p in reordered):
            return None

        for i, pos in enumerate(reordered):
            lower = joint_limits_lower[i] if i < len(joint_limits_lower) else -math.inf
            upper = joint_limits_upper[i] if i < len(joint_limits_upper) else math.inf
            if math.isfinite(lower) and pos < lower:
                if reject_out_of_bounds:
                    return None
                reordered[i] = lower
            if math.isfinite(upper) and pos > upper:
                if reject_out_of_bounds:
                    return None
                reordered[i] = upper

        normalized.positions = reordered

        if point.velocities and len(point.velocities) >= len(msg.joint_names):
            vel = [float(point.velocities[index_by_name[name]]) for name in joint_names]
            if all(math.isfinite(v) for v in vel):
                normalized.velocities = vel

        if point.accelerations and len(point.accelerations) >= len(msg.joint_names):
            acc = [float(point.accelerations[index_by_name[name]]) for name in joint_names]
            if all(math.isfinite(a) for a in acc):
                normalized.accelerations = acc

        normalized.time_from_start = point.time_from_start
        out.points.append(normalized)

    if not out.points:
        return None
    return out


def decode_pose_target(msg: PoseStamped) -> Optional[PoseStamped]:
    """PoseStamped -> PoseStamped, validated passthrough."""
    if not pose_utils.is_valid_absolute_pose(msg):
        return None
    return msg


def decode_pose_chunk(msg: PoseArray) -> Optional[PoseArray]:
    """PoseArray -> PoseArray, validated passthrough."""
    if not msg.poses or len(msg.poses) < 1:
        return None
    return msg


def decode_delta_pose(msg: PoseStamped, ref_pose) -> Optional[PoseStamped]:
    """Delta PoseStamped -> absolute PoseStamped."""
    if ref_pose is None:
        return None
    if not pose_utils.is_valid_absolute_pose(msg):
        return None
    delta = pose_utils.pose_stamped_to_array(msg)
    absolute = pose_utils.apply_delta(ref_pose, delta)
    out = PoseStamped()
    out.header = msg.header
    out.pose = pose_utils.array_to_pose(absolute)
    return out


def decode_delta_chunk(msg: PoseArray, ref_pose) -> Optional[PoseArray]:
    """Delta PoseArray -> absolute PoseArray."""
    if ref_pose is None:
        return None
    out = PoseArray()
    out.header = msg.header
    current_ref = ref_pose.copy()
    for pose in msg.poses:
        delta = pose_utils.pose_to_array(pose)
        absolute = pose_utils.apply_delta(current_ref, delta)
        out.poses.append(pose_utils.array_to_pose(absolute))
        current_ref = absolute
    return out


def decode_twist_target(msg: TwistStamped) -> Optional[TwistStamped]:
    """TwistStamped -> TwistStamped, validated passthrough."""
    values = (
        msg.twist.linear.x,
        msg.twist.linear.y,
        msg.twist.linear.z,
        msg.twist.angular.x,
        msg.twist.angular.y,
        msg.twist.angular.z,
    )
    if not all(math.isfinite(float(v)) for v in values):
        return None
    return msg


@dataclass
class SourceState:
    """Runtime state for one action source in the arbitration pool."""

    name: str
    priority: int
    inactive_timeout_s: float

    last_received_stamp: Optional[rclpy.time.Time] = None
    latest_reference: Optional[JointTrajectory] = None
    latest_contract_type: Optional[str] = None

    # Most recent streaming route this source produced (ROUTE_JSPC or
    # ROUTE_TSKPC). Drives JSPC/TSKPC mutual exclusion. JTC goals do not set
    # this — they are a parallel, independent path.
    latest_stream_route: Optional[str] = None

    decode_fail_count: int = 0
    normalize_drop_count: int = 0
    stale_count: int = 0
    invalid_stamp_count: int = 0
    last_normalize_latency_ms: Optional[float] = None
    last_error: str = ""

    consecutive_valid_count: int = 0

    latest_pose_target: Optional[PoseStamped] = None
    latest_pose_chunk: Optional[PoseArray] = None
    latest_twist_target: Optional[TwistStamped] = None
    ref_pose: Optional[Any] = None

    target_sub: Any = None
    chunk_sub: Any = None
    joint_goal_sub: Any = None
    pose_target_sub: Any = None
    pose_chunk_sub: Any = None
    twist_target_sub: Any = None
