#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Pure arbitration helpers for execution manager sources."""

from __future__ import annotations

from typing import Callable, Dict, Optional

import rclpy

from .execution_manager_core import SourceState


def select_winning_source_by(
    sources: Dict[str, SourceState],
    now: rclpy.time.Time,
    stale_timeout_s: float,
    *,
    has_reference: Callable[[SourceState], bool],
) -> Optional[str]:
    """Return the highest-priority fresh source name for one output path.

    A source is eligible when *has_reference* reports a usable reference and it
    has not exceeded either its per-source inactivity timeout or the global
    stale timeout.  Ties on priority are broken by the most recent stamp.

    This is the single arbitration policy shared by the joint, pose, and twist
    paths; callers differ only in the *has_reference* predicate.
    """
    candidates: list[SourceState] = []

    for src in sources.values():
        if not has_reference(src):
            continue
        if src.last_received_stamp is None:
            continue

        elapsed_s = (now - src.last_received_stamp).nanoseconds / 1e9
        if elapsed_s > src.inactive_timeout_s:
            continue
        if elapsed_s > stale_timeout_s:
            continue

        candidates.append(src)

    if not candidates:
        return None

    candidates.sort(
        key=lambda s: (s.priority, s.last_received_stamp.nanoseconds),
        reverse=True,
    )
    return candidates[0].name


def _has_joint_reference(src: SourceState) -> bool:
    return src.latest_reference is not None


def _has_pose_reference(src: SourceState) -> bool:
    return src.latest_pose_target is not None or src.latest_pose_chunk is not None


def _has_twist_reference(src: SourceState) -> bool:
    return src.latest_twist_target is not None


def _has_any_streaming_reference(src: SourceState) -> bool:
    return (
        src.latest_reference is not None
        or src.latest_pose_target is not None
        or src.latest_pose_chunk is not None
        or src.latest_twist_target is not None
    )


def select_active_streaming_source(
    sources: Dict[str, SourceState],
    now: rclpy.time.Time,
    stale_timeout_s: float,
) -> Optional[str]:
    """Return the single winning streaming source across all routes, or None.

    This enforces JSPC/TSKPC mutual exclusion: only the highest-priority fresh
    source (across joint, pose, and twist references) is allowed to drive its
    route; the other streaming route is suppressed. JTC goals are a parallel
    path and never set a streaming reference, so they are excluded here.
    """
    return select_winning_source_by(
        sources, now, stale_timeout_s, has_reference=_has_any_streaming_reference
    )


def select_winning_source(
    sources: Dict[str, SourceState],
    now: rclpy.time.Time,
    stale_timeout_s: float,
) -> Optional[str]:
    """Return the highest-priority fresh joint-reference source, or None."""
    return select_winning_source_by(
        sources, now, stale_timeout_s, has_reference=_has_joint_reference
    )


def select_winning_pose_source(
    sources: Dict[str, SourceState],
    now: rclpy.time.Time,
    stale_timeout_s: float,
) -> Optional[str]:
    """Return the highest-priority fresh pose-reference source, or None."""
    return select_winning_source_by(
        sources, now, stale_timeout_s, has_reference=_has_pose_reference
    )


def select_winning_twist_source(
    sources: Dict[str, SourceState],
    now: rclpy.time.Time,
    stale_timeout_s: float,
) -> Optional[str]:
    """Return the highest-priority fresh twist-reference source, or None."""
    return select_winning_source_by(
        sources, now, stale_timeout_s, has_reference=_has_twist_reference
    )


def should_switch_to(
    candidate_name: Optional[str],
    current_winner: Optional[str],
    sources: Dict[str, SourceState],
    hold_until: Optional[rclpy.time.Time],
    now: rclpy.time.Time,
) -> bool:
    """Return whether output should switch to *candidate_name*.

    During the anti-flap hold window, only strictly higher-priority sources may
    preempt the current winner.
    """
    if candidate_name is None:
        return False
    if current_winner is None:
        return True
    if candidate_name == current_winner:
        return True

    if hold_until is not None and (hold_until - now).nanoseconds > 0:
        candidate_prio = sources[candidate_name].priority
        current_prio = sources[current_winner].priority
        return candidate_prio > current_prio

    return True
