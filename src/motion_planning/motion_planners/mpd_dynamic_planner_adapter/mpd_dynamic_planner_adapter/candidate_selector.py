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


@dataclass(frozen=True)
class SwitchDecision:
    candidate_index: int | None
    reason: str
    old_cost: float
    new_cost: float
    improvement: float


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
) -> float:
    if not window_end_unix_s > window_start_unix_s or sample_dt_s <= 0.0:
        raise ValueError("comparison window is invalid")
    times, positions, velocities, accelerations = _arrays(result)
    relative_start = window_start_unix_s - trajectory_start_unix_s
    relative_end = window_end_unix_s - trajectory_start_unix_s
    if relative_start < times[0] - 1e-9 or relative_end > times[-1] + 1e-9:
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
    jerk = np.gradient(ddq, sample_times, axis=0, edge_order=1)
    duration = relative_end - relative_start
    path = float(np.sum(np.linalg.norm(np.diff(q, axis=0), axis=1)))
    path_term = path / max(1e-9, max_velocity_rad_s * duration * math.sqrt(q.shape[1]))
    velocity_term = float(np.mean(np.square(dq / max_velocity_rad_s)))
    acceleration_term = float(np.mean(np.square(ddq / max_acceleration_rad_s2)))
    jerk_term = float(np.mean(np.square(jerk / max_jerk_rad_s3)))
    return path_term + velocity_term + acceleration_term + jerk_term


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
    if improvement < switching_hysteresis:
        return SwitchDecision(None, "switching_hysteresis", old_cost, best.total, improvement)
    return SwitchDecision(best.index, "composite_cost_improved", old_cost, best.total, improvement)
