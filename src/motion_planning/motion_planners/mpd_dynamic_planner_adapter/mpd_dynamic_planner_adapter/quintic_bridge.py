"""Limit-aware quintic bridge for moving Phase-4 trajectory replacement."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from manipulation_motion_planning.contracts import (
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)
from mpd_planner_adapter.trajectory import TimedPlan

from .collision_guard import DynamicTrajectoryGuard, TimedCollisionPlan
from .dynamic_world import DynamicWorldSnapshot


class QuinticBridgeError(ValueError):
    """The requested boundary states cannot be connected within the limits."""


@dataclass(frozen=True)
class QuinticBridgeStats:
    duration_s: float
    max_position_change_rad: float
    max_velocity_rad_s: float
    max_acceleration_rad_s2: float
    max_jerk_rad_s3: float
    velocity_utilization: float
    acceleration_utilization: float
    jerk_utilization: float


@dataclass(frozen=True)
class QuinticHandoffChoice:
    handoff_unix_s: float | None
    bridge_duration_s: float | None
    reason: str
    candidates_checked: int


def _vector(values, size: int, name: str) -> np.ndarray:
    output = np.asarray(values, dtype=np.float64)
    if output.shape == ():
        output = np.full(size, float(output), dtype=np.float64)
    if output.shape != (size,) or not np.isfinite(output).all():
        raise QuinticBridgeError(f"{name} must be finite scalar or [{size}] vector")
    return output


def _coefficients(q0, dq0, ddq0, q1, dq1, ddq1, duration_s: float) -> np.ndarray:
    duration = float(duration_s)
    if not math.isfinite(duration) or duration <= 0.0:
        raise QuinticBridgeError("bridge duration must be positive")
    q0, dq0, ddq0, q1, dq1, ddq1 = (
        np.asarray(value, dtype=np.float64)
        for value in (q0, dq0, ddq0, q1, dq1, ddq1)
    )
    if not all(value.shape == q0.shape for value in (dq0, ddq0, q1, dq1, ddq1)):
        raise QuinticBridgeError("bridge boundary arrays have inconsistent shapes")
    c0 = q0
    c1 = dq0
    c2 = 0.5 * ddq0
    position_residual = q1 - (c0 + c1 * duration + c2 * duration**2)
    velocity_residual = dq1 - (c1 + 2.0 * c2 * duration)
    acceleration_residual = ddq1 - 2.0 * c2
    c3 = (
        10.0 * position_residual / duration**3
        - 4.0 * velocity_residual / duration**2
        + 0.5 * acceleration_residual / duration
    )
    c4 = (
        -15.0 * position_residual / duration**4
        + 7.0 * velocity_residual / duration**3
        - acceleration_residual / duration**2
    )
    c5 = (
        6.0 * position_residual / duration**5
        - 3.0 * velocity_residual / duration**4
        + 0.5 * acceleration_residual / duration**3
    )
    return np.stack((c0, c1, c2, c3, c4, c5), axis=0)


def sample_quintic(
    q0,
    dq0,
    ddq0,
    q1,
    dq1,
    ddq1,
    duration_s: float,
    *,
    sample_dt_s: float = 0.02,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if sample_dt_s <= 0.0:
        raise QuinticBridgeError("sample_dt_s must be positive")
    coefficients = _coefficients(q0, dq0, ddq0, q1, dq1, ddq1, duration_s)
    count = max(2, int(math.ceil(float(duration_s) / sample_dt_s)) + 1)
    times = np.linspace(0.0, float(duration_s), count)
    powers = np.stack([times**degree for degree in range(6)], axis=1)
    position = powers @ coefficients
    velocity = powers[:, :5] @ np.stack(
        [degree * coefficients[degree] for degree in range(1, 6)], axis=0
    )
    acceleration = powers[:, :4] @ np.stack(
        [degree * (degree - 1) * coefficients[degree] for degree in range(2, 6)], axis=0
    )
    jerk = powers[:, :3] @ np.stack(
        [degree * (degree - 1) * (degree - 2) * coefficients[degree] for degree in range(3, 6)],
        axis=0,
    )
    return times, position, velocity, acceleration, jerk


def bridge_stats(
    q0,
    dq0,
    ddq0,
    q1,
    dq1,
    ddq1,
    duration_s: float,
    *,
    max_velocity_rad_s,
    max_acceleration_rad_s2,
    max_jerk_rad_s3,
    sample_dt_s: float = 0.02,
) -> QuinticBridgeStats:
    times, position, velocity, acceleration, jerk = sample_quintic(
        q0, dq0, ddq0, q1, dq1, ddq1, duration_s, sample_dt_s=sample_dt_s
    )
    del times
    size = position.shape[1]
    velocity_limits = _vector(max_velocity_rad_s, size, "max_velocity_rad_s")
    acceleration_limits = _vector(max_acceleration_rad_s2, size, "max_acceleration_rad_s2")
    jerk_limits = _vector(max_jerk_rad_s3, size, "max_jerk_rad_s3")
    if np.any(velocity_limits <= 0.0) or np.any(acceleration_limits <= 0.0) or np.any(jerk_limits <= 0.0):
        raise QuinticBridgeError("bridge kinematic limits must be positive")
    return QuinticBridgeStats(
        duration_s=float(duration_s),
        max_position_change_rad=float(np.max(np.abs(position[-1] - position[0]))),
        max_velocity_rad_s=float(np.max(np.abs(velocity))),
        max_acceleration_rad_s2=float(np.max(np.abs(acceleration))),
        max_jerk_rad_s3=float(np.max(np.abs(jerk))),
        velocity_utilization=float(np.max(np.abs(velocity) / velocity_limits)),
        acceleration_utilization=float(np.max(np.abs(acceleration) / acceleration_limits)),
        jerk_utilization=float(np.max(np.abs(jerk) / jerk_limits)),
    )


def minimum_bridge_duration(
    q0,
    dq0,
    ddq0,
    q1,
    dq1,
    ddq1,
    *,
    minimum_duration_s: float,
    maximum_duration_s: float,
    max_velocity_rad_s,
    max_acceleration_rad_s2,
    max_jerk_rad_s3,
    sample_dt_s: float = 0.02,
    tolerance_s: float = 0.002,
) -> tuple[float, QuinticBridgeStats]:
    minimum = float(minimum_duration_s)
    maximum = float(maximum_duration_s)
    if not 0.0 < minimum <= maximum:
        raise QuinticBridgeError("bridge duration bounds are invalid")

    def evaluate(duration: float) -> QuinticBridgeStats:
        return bridge_stats(
            q0,
            dq0,
            ddq0,
            q1,
            dq1,
            ddq1,
            duration,
            max_velocity_rad_s=max_velocity_rad_s,
            max_acceleration_rad_s2=max_acceleration_rad_s2,
            max_jerk_rad_s3=max_jerk_rad_s3,
            sample_dt_s=sample_dt_s,
        )

    high_stats = evaluate(maximum)
    if max(
        high_stats.velocity_utilization,
        high_stats.acceleration_utilization,
        high_stats.jerk_utilization,
    ) > 1.0 + 1e-9:
        raise QuinticBridgeError("no limit-compliant quintic bridge within maximum_duration_s")
    low_stats = evaluate(minimum)
    if max(
        low_stats.velocity_utilization,
        low_stats.acceleration_utilization,
        low_stats.jerk_utilization,
    ) <= 1.0 + 1e-9:
        return minimum, low_stats

    low, high = minimum, maximum
    while high - low > tolerance_s:
        middle = 0.5 * (low + high)
        stats = evaluate(middle)
        if max(
            stats.velocity_utilization,
            stats.acceleration_utilization,
            stats.jerk_utilization,
        ) <= 1.0 + 1e-9:
            high, high_stats = middle, stats
        else:
            low = middle
    return high, high_stats


def select_quintic_handoff(
    *,
    active_plan: TimedPlan,
    collision_plan: TimedCollisionPlan,
    world: DynamicWorldSnapshot,
    guard: DynamicTrajectoryGuard,
    now_unix_s: float,
    bridge_start_unix_s: float,
    latest_handoff_unix_s: float,
    step_s: float,
    minimum_duration_s: float,
    max_velocity_rad_s,
    max_acceleration_rad_s2,
    max_jerk_rad_s3,
    sample_dt_s: float,
) -> QuinticHandoffChoice:
    if step_s <= 0.0:
        raise QuinticBridgeError("handoff step must be positive")
    active_end = active_plan.start_unix_s + active_plan.result.points[-1].time_from_start_s
    latest = min(
        float(latest_handoff_unix_s),
        float(active_end),
        float(collision_plan.absolute_times_s[-1]),
        world.valid_until_unix_ns * 1e-9,
    )
    initial = active_plan.predict_point(bridge_start_unix_s)
    candidate = bridge_start_unix_s + minimum_duration_s
    checked = 0
    while candidate <= latest + 1e-9:
        checked += 1
        target = active_plan.predict_point(candidate)
        available = candidate - bridge_start_unix_s
        try:
            required, _ = minimum_bridge_duration(
                initial.positions,
                initial.velocities,
                initial.accelerations,
                target.positions,
                target.velocities,
                target.accelerations,
                minimum_duration_s=minimum_duration_s,
                maximum_duration_s=available,
                max_velocity_rad_s=max_velocity_rad_s,
                max_acceleration_rad_s2=max_acceleration_rad_s2,
                max_jerk_rad_s3=max_jerk_rad_s3,
                sample_dt_s=sample_dt_s,
            )
        except QuinticBridgeError:
            candidate += step_s
            continue
        if required <= available + 1e-9:
            risk = guard.validate(collision_plan, world, now_unix_s, candidate)
            if risk.safe:
                return QuinticHandoffChoice(
                    candidate,
                    available,
                    "earliest_dynamic_safe_quintic_bridge",
                    checked,
                )
        candidate += step_s
    return QuinticHandoffChoice(None, None, "no_dynamic_safe_quintic_bridge", checked)


def splice_with_quintic_bridge(
    *,
    current_state: StartState,
    current_acceleration,
    new_plan: TrajectoryPlanResult,
    duration_s: float,
    sample_dt_s: float,
    max_velocity_rad_s,
    max_acceleration_rad_s2,
    max_jerk_rad_s3,
) -> TrajectoryPlanResult:
    if not new_plan.valid or not new_plan.joint_names or len(new_plan.points) < 2:
        raise QuinticBridgeError("new plan is invalid or too short")
    if tuple(current_state.joint_names) != tuple(new_plan.joint_names):
        raise QuinticBridgeError("current/new joint names differ")
    first = new_plan.points[0]
    size = len(first.positions)
    q0 = _vector(current_state.positions, size, "current positions")
    dq0 = _vector(current_state.velocities or np.zeros(size), size, "current velocities")
    ddq0 = _vector(current_acceleration, size, "current accelerations")
    q1 = _vector(first.positions, size, "new positions")
    dq1 = _vector(first.velocities or np.zeros(size), size, "new velocities")
    ddq1 = _vector(first.accelerations or np.zeros(size), size, "new accelerations")
    times, positions, velocities, accelerations, _ = sample_quintic(
        q0, dq0, ddq0, q1, dq1, ddq1, duration_s, sample_dt_s=sample_dt_s
    )
    stats = bridge_stats(
        q0,
        dq0,
        ddq0,
        q1,
        dq1,
        ddq1,
        duration_s,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
        sample_dt_s=sample_dt_s,
    )
    if max(stats.velocity_utilization, stats.acceleration_utilization, stats.jerk_utilization) > 1.0 + 1e-9:
        raise QuinticBridgeError("quintic bridge exceeds configured kinematic limits")
    merged = [
        TrajectoryPlanPoint(
            positions=positions[index].tolist(),
            velocities=velocities[index].tolist(),
            accelerations=accelerations[index].tolist(),
            time_from_start_s=float(times[index]),
        )
        for index in range(len(times))
    ]
    for point in new_plan.points[1:]:
        merged.append(
            TrajectoryPlanPoint(
                positions=list(point.positions),
                velocities=None if point.velocities is None else list(point.velocities),
                accelerations=None if point.accelerations is None else list(point.accelerations),
                time_from_start_s=float(duration_s + point.time_from_start_s),
            )
        )
    return TrajectoryPlanResult(
        valid=True,
        joint_names=list(new_plan.joint_names),
        points=merged,
        diagnostics={
            **new_plan.diagnostics,
            "bridge": {
                "type": "quintic",
                **stats.__dict__,
                "points": len(times),
            },
        },
    )
