#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Decode and SourceState update helpers for execution manager ingest paths.

The helpers in this module are intentionally lightweight: no ROS node access,
no publishers, no subscriptions, no logging.  They keep contract dispatch and
SourceState mutation testable without adding runtime scheduling work.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from trajectory_msgs.msg import JointTrajectory

from . import pose_utils
from .execution_manager_core import (
    CONTRACT_DELTA_CHUNK,
    CONTRACT_DELTA_POSE,
    CONTRACT_JOINT_TARGET,
    CONTRACT_POSE_TARGET,
    CONTRACT_TWIST_TARGET,
    SourceState,
    decode_delta_chunk,
    decode_delta_pose,
    decode_joint_chunk,
    decode_joint_target,
    decode_pose_chunk,
    decode_pose_target,
    decode_twist_target,
)


def decode_joint_reference(
    *,
    contract_type: str,
    msg,
    joint_names: list[str],
    joint_limits_lower: list[float],
    joint_limits_upper: list[float],
    clock: rclpy.clock.Clock,
    reject_zero_stamp: bool,
) -> Optional[JointTrajectory]:
    """Decode a joint-space EM contract into a normalized JointTrajectory."""
    if contract_type == CONTRACT_JOINT_TARGET:
        return decode_joint_target(
            msg,
            joint_names,
            joint_limits_lower,
            joint_limits_upper,
            reject_out_of_bounds=False,
            clock=clock,
            reject_zero_stamp=reject_zero_stamp,
        )

    return decode_joint_chunk(
        msg,
        joint_names,
        joint_limits_lower,
        joint_limits_upper,
        reject_out_of_bounds=False,
    )


def joint_reference_drop_reason(normalized: Optional[JointTrajectory]) -> Optional[str]:
    """Return a normalization drop reason, or None when the reference is usable."""
    if normalized is None:
        return "decode failed"
    if not normalized.joint_names or not normalized.points:
        return "normalize drop: empty joint_names or points"
    return None


def apply_valid_joint_reference(
    *,
    source: SourceState,
    normalized: JointTrajectory,
    contract_type: str,
    now: rclpy.time.Time,
    latency_ms: Optional[float],
) -> None:
    """Store a validated joint reference in SourceState."""
    source.latest_reference = normalized
    source.latest_contract_type = contract_type
    source.last_received_stamp = now
    source.consecutive_valid_count += 1
    if latency_ms is not None:
        source.last_normalize_latency_ms = max(0.0, latency_ms)


def decode_pose_reference(
    *,
    contract_type: str,
    msg,
    source_ref_pose,
) -> tuple[Optional[PoseStamped | PoseArray], bool, object]:
    """Decode a pose-space EM contract.

    Returns ``(decoded, is_chunk, ref_pose_update)``.  ``ref_pose_update`` is
    non-None only when an absolute pose initializes delta tracking.
    """
    if contract_type in (CONTRACT_POSE_TARGET, CONTRACT_DELTA_POSE):
        if contract_type == CONTRACT_DELTA_POSE:
            return decode_delta_pose(msg, source_ref_pose), False, None

        decoded = decode_pose_target(msg)
        ref_pose_update = None
        if decoded is not None and source_ref_pose is None:
            ref_pose_update = pose_utils.pose_stamped_to_array(msg)
        return decoded, False, ref_pose_update

    if contract_type == CONTRACT_DELTA_CHUNK:
        return decode_delta_chunk(msg, source_ref_pose), True, None

    return decode_pose_chunk(msg), True, None


def apply_valid_pose_reference(
    *,
    source: SourceState,
    decoded: PoseStamped | PoseArray,
    is_chunk: bool,
    now: rclpy.time.Time,
    ref_pose_update,
    latency_ms: Optional[float],
) -> None:
    """Store a validated pose reference in SourceState."""
    if ref_pose_update is not None:
        source.ref_pose = ref_pose_update

    if is_chunk:
        source.latest_pose_chunk = decoded
        source.latest_pose_target = None
    else:
        source.latest_pose_target = decoded
        source.latest_pose_chunk = None

    source.last_received_stamp = now
    if latency_ms is not None:
        source.last_normalize_latency_ms = max(0.0, latency_ms)
    source.consecutive_valid_count += 1


def decode_twist_reference(
    *,
    contract_type: str,
    msg,
) -> Optional[TwistStamped]:
    """Decode a TwistStamped EM contract."""
    if contract_type != CONTRACT_TWIST_TARGET:
        return None
    return decode_twist_target(msg)


def apply_valid_twist_reference(
    *,
    source: SourceState,
    decoded: TwistStamped,
    now: rclpy.time.Time,
    latency_ms: Optional[float],
) -> None:
    """Store a validated twist reference in SourceState."""
    source.latest_twist_target = decoded
    source.last_received_stamp = now
    if latency_ms is not None:
        source.last_normalize_latency_ms = max(0.0, latency_ms)
    source.consecutive_valid_count += 1
