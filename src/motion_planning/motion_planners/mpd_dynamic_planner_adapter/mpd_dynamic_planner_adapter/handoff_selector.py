"""Earliest feasible low-speed dynamic handoff selection."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from mpd_planner_adapter.trajectory import TimedPlan

from .collision_guard import DynamicTrajectoryGuard, TimedCollisionPlan
from .dynamic_world import DynamicWorldSnapshot


@dataclass(frozen=True)
class HandoffChoice:
    handoff_unix_s: float | None
    reason: str
    candidates_checked: int


def select_earliest_low_speed_handoff(
    *,
    active_plan: TimedPlan,
    collision_plan: TimedCollisionPlan,
    world: DynamicWorldSnapshot,
    guard: DynamicTrajectoryGuard,
    now_unix_s: float,
    earliest_unix_s: float,
    latest_unix_s: float,
    step_s: float = 0.05,
    max_speed_rad_s: float = 0.20,
) -> HandoffChoice:
    if step_s <= 0.0:
        raise ValueError("step_s must be positive")
    active_end = active_plan.start_unix_s + active_plan.result.points[-1].time_from_start_s
    latest = min(
        latest_unix_s,
        float(active_end),
        float(collision_plan.absolute_times_s[-1]),
        world.valid_until_unix_ns * 1e-9,
    )
    candidate = max(earliest_unix_s, now_unix_s)
    checked = 0
    while candidate <= latest + 1e-9:
        checked += 1
        state = active_plan.predict(candidate)
        speed = float(np.max(np.abs(state.velocities or np.zeros(len(state.positions)))))
        if speed <= max_speed_rad_s:
            risk = guard.validate(collision_plan, world, now_unix_s, candidate)
            if risk.safe:
                return HandoffChoice(candidate, "earliest_dynamic_safe_low_speed", checked)
        candidate += step_s
    return HandoffChoice(None, "no_dynamic_safe_low_speed_handoff", checked)
