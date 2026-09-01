"""Common-window cost and hysteretic top-K replacement selection."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from manipulation_motion_planning.contracts import TrajectoryPlanResult


@dataclass(frozen=True)
class CandidateCost:
    index: int
    total: float
    kinematic: float
    clearance: float
    mpd: float
    bridge: float
    tail_kinematic: float = 0.0
    deviation: float = 0.0


@dataclass(frozen=True)
class SwitchDecision:
    candidate_index: int | None
    reason: str
    old_cost: float
    new_cost: float
    improvement: float


@dataclass(frozen=True)
class AdaptiveDeviationWeight:
    effective_weight: float
    clearance_gate: float
    ttc_gate: float
    predicted_ttc_s: float


def _smoothstep_gate(value: float, zero_at: float, full_at: float) -> float:
    if not full_at > zero_at:
        raise ValueError("adaptive deviation thresholds must be strictly ordered")
    if math.isnan(value):
        return 0.0
    if value <= zero_at:
        return 0.0
    if value >= full_at:
        return 1.0
    normalized = (value - zero_at) / (full_at - zero_at)
    return normalized * normalized * (3.0 - 2.0 * normalized)


def adaptive_deviation_weight(
    base_weight: float,
    *,
    minimum_clearance_m: float,
    first_collision_unix_s: float | None,
    reference_unix_s: float,
    old_hard_safe: bool,
    clearance_zero_m: float,
    clearance_full_m: float,
    ttc_zero_s: float,
    ttc_full_s: float,
    enabled: bool = True,
) -> AdaptiveDeviationWeight:
    """Fade continuity preference as the active trajectory becomes risky.

    Clearance and time-to-collision use smoothstep gates. The more conservative
    gate wins, and a hard collision on the active motion removes the deviation
    preference completely.
    """

    if base_weight < 0.0 or not math.isfinite(base_weight):
        raise ValueError("base deviation weight must be finite and non-negative")
    predicted_ttc_s = (
        math.inf
        if first_collision_unix_s is None
        else float(first_collision_unix_s) - float(reference_unix_s)
    )
    clearance_gate = _smoothstep_gate(
        float(minimum_clearance_m), clearance_zero_m, clearance_full_m
    )
    ttc_gate = _smoothstep_gate(predicted_ttc_s, ttc_zero_s, ttc_full_s)
    risk_gate = (
        (min(clearance_gate, ttc_gate) if old_hard_safe else 0.0)
        if enabled
        else 1.0
    )
    return AdaptiveDeviationWeight(
        effective_weight=base_weight * risk_gate,
        clearance_gate=clearance_gate,
        ttc_gate=ttc_gate,
        predicted_ttc_s=predicted_ttc_s,
    )


def _arrays(result: TrajectoryPlanResult) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    times = np.asarray([point.time_from_start_s for point in result.points], dtype=np.float64)
    positions = np.asarray([point.positions for point in result.points], dtype=np.float64)
    velocities = np.asarray(
        [
            point.velocities if point.velocities is not None else np.zeros(positions.shape[1])
            for point in result.points
        ],
        dtype=np.float64,
    )
    accelerations = np.asarray(
        [
            point.accelerations
            if point.accelerations is not None
            else np.zeros(positions.shape[1])
            for point in result.points
        ],
        dtype=np.float64,
    )
    if (
        positions.ndim != 2
        or len(times) < 2
        or velocities.shape != positions.shape
        or accelerations.shape != positions.shape
        or np.any(np.diff(times) <= 0.0)
        or not all(np.isfinite(value).all() for value in (times, positions, velocities, accelerations))
    ):
        raise ValueError("trajectory arrays are invalid")
    return times, positions, velocities, accelerations


def common_window_kinematic_cost(
    result: TrajectoryPlanResult,
    *,
    trajectory_start_unix_s: float,
    window_start_unix_s: float,
    window_end_unix_s: float,
    max_velocity_rad_s: float,
    max_acceleration_rad_s2: float,
    max_jerk_rad_s3: float,
    sample_dt_s: float = 0.02,
    hold_after_end: bool = False,
) -> float:
    if not window_end_unix_s > window_start_unix_s or sample_dt_s <= 0.0:
        raise ValueError("comparison window is invalid")
    times, positions, velocities, accelerations = _arrays(result)
    relative_start = window_start_unix_s - trajectory_start_unix_s
    relative_end = window_end_unix_s - trajectory_start_unix_s
    if relative_start < times[0] - 1e-9 or (
        relative_end > times[-1] + 1e-9 and not hold_after_end
    ):
        return math.inf
    count = max(2, int(math.ceil((relative_end - relative_start) / sample_dt_s)) + 1)
    sample_times = np.linspace(relative_start, relative_end, count)
    q = np.column_stack(
        [np.interp(sample_times, times, positions[:, joint]) for joint in range(positions.shape[1])]
    )
    dq = np.column_stack(
        [np.interp(sample_times, times, velocities[:, joint]) for joint in range(positions.shape[1])]
    )
    ddq = np.column_stack(
        [np.interp(sample_times, times, accelerations[:, joint]) for joint in range(positions.shape[1])]
    )
    if hold_after_end:
        held = sample_times > times[-1] + 1e-9
        q[held] = positions[-1]
        dq[held] = 0.0
        ddq[held] = 0.0
    jerk = np.gradient(ddq, sample_times, axis=0, edge_order=1)
    duration = relative_end - relative_start
    path = float(np.sum(np.linalg.norm(np.diff(q, axis=0), axis=1)))
    path_term = path / max(1e-9, max_velocity_rad_s * duration * math.sqrt(q.shape[1]))
    velocity_term = float(np.mean(np.square(dq / max_velocity_rad_s)))
    acceleration_term = float(np.mean(np.square(ddq / max_acceleration_rad_s2)))
    jerk_term = float(np.mean(np.square(jerk / max_jerk_rad_s3)))
    return path_term + velocity_term + acceleration_term + jerk_term


def common_window_deviation_cost(
    new_result: TrajectoryPlanResult,
    old_result: TrajectoryPlanResult,
    *,
    new_trajectory_start_unix_s: float,
    old_trajectory_start_unix_s: float,
    window_start_unix_s: float,
    window_end_unix_s: float,
    sample_dt_s: float = 0.02,
    hold_after_end: bool = True,
) -> float:
    """Mean time-aligned squared joint deviation between two alternatives."""

    if not window_end_unix_s > window_start_unix_s or sample_dt_s <= 0.0:
        raise ValueError("comparison window is invalid")
    new_times, new_positions, _, _ = _arrays(new_result)
    old_times, old_positions, _, _ = _arrays(old_result)
    if new_positions.shape[1] != old_positions.shape[1]:
        raise ValueError("trajectory joint dimensions do not match")
    count = max(
        2,
        int(math.ceil((window_end_unix_s - window_start_unix_s) / sample_dt_s))
        + 1,
    )
    absolute_times = np.linspace(window_start_unix_s, window_end_unix_s, count)

    def sample(times, positions, trajectory_start):
        relative = absolute_times - trajectory_start
        if relative[0] < times[0] - 1e-9 or (
            relative[-1] > times[-1] + 1e-9 and not hold_after_end
        ):
            return None
        return np.column_stack(
            [
                np.interp(relative, times, positions[:, joint])
                for joint in range(positions.shape[1])
            ]
        )

    new_q = sample(new_times, new_positions, new_trajectory_start_unix_s)
    old_q = sample(old_times, old_positions, old_trajectory_start_unix_s)
    if new_q is None or old_q is None:
        return math.inf
    return float(np.mean(np.sum(np.square(new_q - old_q), axis=1)))


def clearance_cost(minimum_clearance_m: float, preferred_clearance_m: float) -> float:
    if minimum_clearance_m <= 0.0:
        return math.inf
    if preferred_clearance_m <= 0.0 or minimum_clearance_m >= preferred_clearance_m:
        return 0.0
    return ((preferred_clearance_m - minimum_clearance_m) / preferred_clearance_m) ** 2


def choose_hysteretic_switch(
    candidates: list[CandidateCost],
    *,
    old_cost: float,
    old_safe: bool,
    minimum_commit_interval_elapsed: bool,
    switching_hysteresis: float,
    relative_hysteresis: float = 0.0,
    forced_switch_reason: str | None = None,
) -> SwitchDecision:
    finite = [candidate for candidate in candidates if math.isfinite(candidate.total)]
    if not finite:
        return SwitchDecision(None, "no_latest_world_safe_candidate", old_cost, math.inf, -math.inf)
    best = min(finite, key=lambda candidate: candidate.total)
    if not old_safe or not math.isfinite(old_cost):
        return SwitchDecision(best.index, "old_trajectory_unsafe", old_cost, best.total, math.inf)
    improvement = old_cost - best.total
    if not minimum_commit_interval_elapsed:
        return SwitchDecision(None, "minimum_commit_interval", old_cost, best.total, improvement)
    if forced_switch_reason is not None:
        return SwitchDecision(
            best.index,
            forced_switch_reason,
            old_cost,
            best.total,
            improvement,
        )
    required_improvement = max(
        switching_hysteresis,
        relative_hysteresis * abs(old_cost),
    )
    # Preserve Phase 4's equality behavior when relative hysteresis is disabled.
    hysteresis_rejects = improvement < required_improvement or (
        relative_hysteresis > 0.0 and improvement <= required_improvement
    )
    if hysteresis_rejects:
        return SwitchDecision(None, "switching_hysteresis", old_cost, best.total, improvement)
    return SwitchDecision(best.index, "composite_cost_improved", old_cost, best.total, improvement)
