"""Bounded joint-space braking trajectory used when no safe handoff exists."""

from __future__ import annotations

import math

import numpy as np

from manipulation_motion_planning.contracts import (
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)


def make_braking_plan(
    state: StartState,
    *,
    max_deceleration_rad_s2: float = 1.0,
    minimum_duration_s: float = 0.20,
    sample_dt_s: float = 0.02,
) -> TrajectoryPlanResult:
    if max_deceleration_rad_s2 <= 0.0 or minimum_duration_s <= 0.0 or sample_dt_s <= 0.0:
        raise ValueError("braking limits must be positive")
    q0 = np.asarray(state.positions, dtype=np.float64)
    dq0 = np.asarray(state.velocities or np.zeros_like(q0), dtype=np.float64)
    if q0.ndim != 1 or dq0.shape != q0.shape or not np.isfinite(q0).all() or not np.isfinite(dq0).all():
        raise ValueError("braking state arrays are invalid")
    duration = max(minimum_duration_s, float(np.max(np.abs(dq0))) / max_deceleration_rad_s2)
    count = max(2, int(math.ceil(duration / sample_dt_s)) + 1)
    times = np.linspace(0.0, duration, count)
    points = []
    for stamp in times:
        alpha = stamp / duration
        velocity = dq0 * (1.0 - alpha)
        position = q0 + dq0 * (stamp - 0.5 * stamp**2 / duration)
        points.append(
            TrajectoryPlanPoint(
                positions=position.tolist(),
                velocities=velocity.tolist(),
                time_from_start_s=float(stamp),
            )
        )
    return TrajectoryPlanResult(
        valid=True,
        joint_names=list(state.joint_names),
        points=points,
        diagnostics={
            "mode": "controlled_brake",
            "duration_s": duration,
            "max_deceleration_rad_s2": float(np.max(np.abs(dq0)) / duration),
        },
    )
