#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for the planner-owned state cache (no rclpy dependency)."""

from __future__ import annotations

from manipulation_motion_planning.state_cache import RobotStateCache, match_joint_state


def test_match_joint_state_reorders_by_name() -> None:
    positions, velocities, missing = match_joint_state(
        msg_names=["b", "a", "c"],
        msg_positions=[2.0, 1.0, 3.0],
        msg_velocities=None,
        joint_names=["a", "b", "c"],
    )
    assert missing == []
    assert positions == [1.0, 2.0, 3.0]
    assert velocities is None


def test_match_joint_state_reports_missing_joint() -> None:
    positions, velocities, missing = match_joint_state(
        msg_names=["a", "b"],
        msg_positions=[1.0, 2.0],
        msg_velocities=None,
        joint_names=["a", "b", "c"],
    )
    assert positions is None
    assert velocities is None
    assert missing == ["c"]


def test_cache_reports_none_before_first_update() -> None:
    cache = RobotStateCache(joint_names=["a", "b"], max_age_s=0.05)
    assert cache.get_fresh(now_s=1.0) is None


def test_cache_returns_state_within_max_age() -> None:
    cache = RobotStateCache(joint_names=["a", "b"], max_age_s=0.05)
    cache.update(["a", "b"], [1.0, 2.0], None, stamp_s=1.000)
    state = cache.get_fresh(now_s=1.020)
    assert state is not None
    assert state.positions == [1.0, 2.0]


def test_cache_drops_state_older_than_max_age() -> None:
    cache = RobotStateCache(joint_names=["a", "b"], max_age_s=0.05)
    cache.update(["a", "b"], [1.0, 2.0], None, stamp_s=1.000)
    assert cache.get_fresh(now_s=1.100) is None


def test_cache_keeps_last_missing_joints_for_diagnostics() -> None:
    cache = RobotStateCache(joint_names=["a", "b", "c"], max_age_s=0.05)
    cache.update(["a", "b"], [1.0, 2.0], None, stamp_s=1.0)
    assert cache.get_fresh(now_s=1.0) is None
    assert cache.last_missing_joints == ["c"]
