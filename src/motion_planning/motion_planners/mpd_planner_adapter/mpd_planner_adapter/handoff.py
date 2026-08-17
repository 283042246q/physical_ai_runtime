"""Pure trajectory splice validation for controlled JTC handoff."""

from __future__ import annotations

import math

import numpy as np

from manipulation_motion_planning.contracts import (
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)

from .trajectory import TimedPlan


class HandoffValidationError(ValueError):
    """The candidate cannot safely replace the active trajectory."""


def _point(state: StartState, relative_s: float) -> TrajectoryPlanPoint:
    return TrajectoryPlanPoint(
        positions=list(state.positions),
        velocities=(None if state.velocities is None else list(state.velocities)),
        time_from_start_s=float(relative_s),
    )


def _max_abs(values) -> float:
    array = np.asarray(values, dtype=np.float64)
    return float(np.max(np.abs(array)))


def splice_for_handoff(
    *,
    current_state: StartState,
    active_plan: TimedPlan | None,
    new_plan: TrajectoryPlanResult,
    commit_start_unix_s: float,
    handoff_unix_s: float,
    prefix_dt_s: float = 0.05,
    max_start_drift_rad: float = 0.10,
    max_handoff_speed_rad_s: float = 0.20,
    max_q_jump_rad: float = 0.03,
    max_dq_jump_rad_s: float = 0.20,
    max_ddq_jump_rad_s2: float = 2.0,
) -> TrajectoryPlanResult:
    """Return old-safe-prefix + new-suffix, or reject before JTC submission."""
    if not new_plan.valid or not new_plan.joint_names or len(new_plan.points) < 2:
        raise HandoffValidationError("new plan is invalid or too short")
    if tuple(current_state.joint_names) != tuple(new_plan.joint_names):
        raise HandoffValidationError("current/new joint names differ")
    prefix_duration = handoff_unix_s - commit_start_unix_s
    if not math.isfinite(prefix_duration) or prefix_duration <= 0.0:
        raise HandoffValidationError("handoff is not in the future")
    if prefix_dt_s <= 0.0:
        raise HandoffValidationError("prefix_dt_s must be positive")

    current = StartState(
        joint_names=list(current_state.joint_names),
        positions=list(current_state.positions),
        velocities=(
            [0.0] * len(current_state.positions)
            if current_state.velocities is None
            else list(current_state.velocities)
        ),
        stamp_s=commit_start_unix_s,
    )
    if active_plan is None:
        prefix = [_point(current, 0.0), _point(current, prefix_duration)]
        old_handoff = current
    else:
        expected_now = active_plan.predict(commit_start_unix_s)
        drift = _max_abs(np.asarray(current.positions) - expected_now.positions)
        if drift > max_start_drift_rad:
            raise HandoffValidationError(
                f"active-plan start drift {drift:.6f} > {max_start_drift_rad:.6f} rad"
            )
        sample_count = max(2, int(math.ceil(prefix_duration / prefix_dt_s)) + 1)
        sample_times = np.linspace(commit_start_unix_s, handoff_unix_s, sample_count)
        prefix = [
            _point(active_plan.predict(float(stamp)), float(stamp - commit_start_unix_s))
            for stamp in sample_times
        ]
        prefix[0] = _point(current, 0.0)
        old_handoff = active_plan.predict(handoff_unix_s)

    new_first = new_plan.points[0]
    old_velocity = np.asarray(old_handoff.velocities or [0.0] * len(new_first.positions))
    new_velocity = np.asarray(new_first.velocities or [0.0] * len(new_first.positions))
    q_jump = _max_abs(np.asarray(old_handoff.positions) - new_first.positions)
    dq_jump = _max_abs(old_velocity - new_velocity)
    handoff_speed = max(_max_abs(old_velocity), _max_abs(new_velocity))
    new_dt = new_plan.points[1].time_from_start_s - new_first.time_from_start_s
    old_dt = prefix[-1].time_from_start_s - prefix[-2].time_from_start_s
    if new_dt <= 0.0 or old_dt <= 0.0:
        raise HandoffValidationError("non-positive splice sample interval")
    old_prev_velocity = np.asarray(prefix[-2].velocities or old_velocity)
    new_next_velocity = np.asarray(new_plan.points[1].velocities or new_velocity)
    old_acceleration = (old_velocity - old_prev_velocity) / old_dt
    new_acceleration = (new_next_velocity - new_velocity) / new_dt
    ddq_jump = _max_abs(old_acceleration - new_acceleration)

    violations = []
    if handoff_speed > max_handoff_speed_rad_s:
        violations.append(
            f"speed {handoff_speed:.6f} > {max_handoff_speed_rad_s:.6f} rad/s"
        )
    if q_jump > max_q_jump_rad:
        violations.append(f"q jump {q_jump:.6f} > {max_q_jump_rad:.6f} rad")
    if dq_jump > max_dq_jump_rad_s:
        violations.append(
            f"dq jump {dq_jump:.6f} > {max_dq_jump_rad_s:.6f} rad/s"
        )
    if ddq_jump > max_ddq_jump_rad_s2:
        violations.append(
            f"ddq jump {ddq_jump:.6f} > {max_ddq_jump_rad_s2:.6f} rad/s^2"
        )
    if violations:
        raise HandoffValidationError("; ".join(violations))

    merged = list(prefix)
    for point in new_plan.points[1:]:
        merged.append(
            TrajectoryPlanPoint(
                positions=list(point.positions),
                velocities=(None if point.velocities is None else list(point.velocities)),
                time_from_start_s=prefix_duration + point.time_from_start_s,
            )
        )
    if any(
        right.time_from_start_s <= left.time_from_start_s
        for left, right in zip(merged, merged[1:])
    ):
        raise HandoffValidationError("merged trajectory time is not increasing")
    return TrajectoryPlanResult(
        valid=True,
        joint_names=list(new_plan.joint_names),
        points=merged,
        diagnostics={
            **new_plan.diagnostics,
            "splice_q_jump_rad": q_jump,
            "splice_dq_jump_rad_s": dq_jump,
            "splice_ddq_jump_rad_s2": ddq_jump,
            "splice_handoff_speed_rad_s": handoff_speed,
            "splice_prefix_points": len(prefix),
        },
    )
