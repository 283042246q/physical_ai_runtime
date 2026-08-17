#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for OnlineMpcPlannerSource (horizon MPC → joint_chunk only)."""

from __future__ import annotations

import pytest
import rclpy
from sensor_msgs.msg import JointState

from manipulation_motion_planning.command_sink import RecordingCommandSink
from manipulation_motion_planning.contracts import (
    CurrentState,
    HorizonPlanPoint,
    HorizonPlanResult,
    JointTarget,
)
from manipulation_motion_planning.planner_sources import (
    OnlineMpcPlannerSource,
    OnlineMpcSourceConfig,
)

if not rclpy.ok():
    rclpy.init()


class _FakeMpcBackend:
    def __init__(self, horizon: int = 4) -> None:
        self.horizon = horizon
        self.warmup_calls = 0
        self.reset_calls: list[CurrentState] = []
        self.next_result: HorizonPlanResult | None = None

    def warmup(self) -> None:
        self.warmup_calls += 1

    def reset(self, current_state: CurrentState) -> None:
        self.reset_calls.append(current_state)

    def update_target(self, target: JointTarget) -> None:
        pass

    def update_world(self, world) -> None:  # noqa: ANN001
        pass

    def step(self, current_state: CurrentState, dt: float) -> HorizonPlanResult:
        if self.next_result is not None:
            return self.next_result
        points = [
            HorizonPlanPoint(
                positions=list(current_state.positions),
                time_from_start_s=step * dt,
            )
            for step in range(self.horizon)
        ]
        return HorizonPlanResult(valid=True, points=points)


def _make_node(
    horizon: int = 4,
) -> tuple[OnlineMpcPlannerSource, _FakeMpcBackend, RecordingCommandSink]:
    backend = _FakeMpcBackend(horizon=horizon)
    sink = RecordingCommandSink()
    node = OnlineMpcPlannerSource("test_horizon_mpc_source")
    node.configure(
        backend=backend,
        joint_names=["j1", "j2"],
        config=OnlineMpcSourceConfig(
            source_name="test_mpc",
            step_rate_hz=50.0,
            max_state_age_s=0.05,
            max_target_age_s=0.1,
        ),
        command_sink=sink,
    )
    return node, backend, sink


def _joint_state(positions: list[float], stamp_s: float) -> JointState:
    msg = JointState()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
    msg.name = ["j1", "j2"]
    msg.position = positions
    return msg


def test_no_publish_without_state() -> None:
    node, _, sink = _make_node()
    try:
        node._on_step_timer()
        assert sink.joint_chunks == []
        assert node._fail_count == 1
    finally:
        node.destroy_node()


def test_publishes_chunk_when_state_and_target_fresh() -> None:
    node, backend, sink = _make_node(horizon=4)
    try:
        now_s = _now_s(node)
        node._on_state(_joint_state([0.1, 0.2], now_s))
        node.set_target(JointTarget(joint_names=["j1", "j2"], positions=[0.3, 0.4]), now_s)
        node._on_step_timer()
        assert len(sink.joint_chunks) == 1
        assert len(sink.joint_chunks[0][1]) == 4
        assert backend.reset_calls
    finally:
        node.destroy_node()


def test_stale_target_blocks_publish() -> None:
    node, _, sink = _make_node()
    try:
        now_s = _now_s(node)
        node._on_state(_joint_state([0.1, 0.2], now_s))
        node.set_target(
            JointTarget(joint_names=["j1", "j2"], positions=[0.3, 0.4]), now_s - 1.0
        )
        node._on_step_timer()
        assert sink.joint_chunks == []
        assert node._last_error == "target_stale"
    finally:
        node.destroy_node()


def test_invalid_backend_result_not_published() -> None:
    node, backend, sink = _make_node()
    backend.next_result = HorizonPlanResult(valid=False, points=[], reason="solver_failed")
    try:
        now_s = _now_s(node)
        node._on_state(_joint_state([0.1, 0.2], now_s))
        node.set_target(JointTarget(joint_names=["j1", "j2"], positions=[0.3, 0.4]), now_s)
        node._on_step_timer()
        assert sink.joint_chunks == []
        assert node._last_error == "solver_failed"
    finally:
        node.destroy_node()


def _now_s(node: OnlineMpcPlannerSource) -> float:
    return node.get_clock().now().nanoseconds / 1e9
