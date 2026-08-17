#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for planner registry and manager."""

from __future__ import annotations

import pytest

from manipulation_motion_planning.planner_manager import (
    PlannerManager,
    PlannerRegistry,
    PlannerSpec,
)


class _Backend:
    def __init__(self) -> None:
        self.warmup_calls = 0

    def warmup(self) -> None:
        self.warmup_calls += 1


def test_registry_creates_and_warms_backend() -> None:
    registry: PlannerRegistry[_Backend] = PlannerRegistry()
    registry.register(PlannerSpec(name="mpc", factory=_Backend))

    backend = registry.create("mpc")

    assert isinstance(backend, _Backend)
    assert backend.warmup_calls == 1


def test_manager_caches_instances_and_switches_active() -> None:
    registry: PlannerRegistry[_Backend] = PlannerRegistry()
    registry.register(PlannerSpec(name="global", factory=_Backend))
    manager = PlannerManager(registry)

    first = manager.switch("global")
    second = manager.get("global")

    assert first is second
    assert manager.active() is first
    assert manager.active_name == "global"


def test_registry_rejects_unknown_planner() -> None:
    registry: PlannerRegistry[_Backend] = PlannerRegistry()

    with pytest.raises(ValueError, match="Unknown planner"):
        registry.create("missing")
