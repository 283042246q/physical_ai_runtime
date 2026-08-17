#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for lightweight robot context facade."""

from __future__ import annotations

from manipulation_motion_planning.robot_context import (
    CachedJointStateProvider,
    RobotContext,
    RobotModelInfo,
    StaticRobotModelProvider,
)


def test_robot_context_returns_model_and_fresh_state() -> None:
    state_provider = CachedJointStateProvider(["j1", "j2"], max_age_s=0.1)
    model_provider = StaticRobotModelProvider(
        RobotModelInfo(
            joint_names=["j1", "j2"],
            base_frame="base",
            tool_frame="tool",
        )
    )
    context = RobotContext(
        state_provider=state_provider,
        model_provider=model_provider,
    )

    state_provider.update(["j2", "j1"], [2.0, 1.0], None, stamp_s=1.0)

    state = context.get_current_state(now_s=1.05)
    assert state is not None
    assert state.positions == [1.0, 2.0]
    assert context.get_model_info().tool_frame == "tool"
    assert not context.get_execution_feedback().faulted


def test_cached_state_provider_reports_missing_joints() -> None:
    state_provider = CachedJointStateProvider(["j1", "j2"], max_age_s=0.1)

    state_provider.update(["j1"], [1.0], None, stamp_s=1.0)

    assert state_provider.get_current_state(now_s=1.0) is None
    assert state_provider.last_missing_joints == ["j2"]
