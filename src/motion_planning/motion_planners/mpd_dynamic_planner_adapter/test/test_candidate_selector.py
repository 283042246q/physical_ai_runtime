import math

from manipulation_motion_planning.contracts import (
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)
from mpd_dynamic_planner_adapter.candidate_selector import (
    CandidateCost,
    choose_hysteretic_switch,
    clearance_cost,
    common_window_kinematic_cost,
)


def _candidate(index, total):
    return CandidateCost(index, total, total, 0.0, 0.0, 0.0)


def test_selects_lowest_composite_cost_when_old_is_unsafe():
    decision = choose_hysteretic_switch(
        [_candidate(0, 0.7), _candidate(1, 0.2)],
        old_cost=math.inf,
        old_safe=False,
        minimum_commit_interval_elapsed=False,
        switching_hysteresis=0.5,
    )

    assert decision.candidate_index == 1
    assert decision.reason == "old_trajectory_unsafe"


def test_hysteresis_and_minimum_interval_keep_safe_old_trajectory():
    interval = choose_hysteretic_switch(
        [_candidate(0, 0.2)],
        old_cost=0.5,
        old_safe=True,
        minimum_commit_interval_elapsed=False,
        switching_hysteresis=0.1,
    )
    hysteresis = choose_hysteretic_switch(
        [_candidate(0, 0.45)],
        old_cost=0.5,
        old_safe=True,
        minimum_commit_interval_elapsed=True,
        switching_hysteresis=0.1,
    )

    assert interval.candidate_index is None
    assert interval.reason == "minimum_commit_interval"
    assert hysteresis.candidate_index is None
    assert hysteresis.reason == "switching_hysteresis"


def test_exhaustion_reserve_overrides_hysteresis_after_minimum_interval():
    decision = choose_hysteretic_switch(
        [_candidate(0, 1.05)],
        old_cost=1.0,
        old_safe=True,
        minimum_commit_interval_elapsed=True,
        switching_hysteresis=0.1,
        forced_switch_reason="old_trajectory_exhaustion_reserve",
    )

    assert decision.candidate_index == 0
    assert decision.reason == "old_trajectory_exhaustion_reserve"


def test_clearance_cost_is_zero_above_preference_and_infinite_at_collision():
    assert clearance_cost(0.2, 0.1) == 0.0
    assert 0.0 < clearance_cost(0.05, 0.1) < 1.0
    assert math.isinf(clearance_cost(0.0, 0.1))


def _trajectory(samples):
    return TrajectoryPlanResult(
        valid=True,
        joint_names=[f"joint_{index}" for index in range(7)],
        points=[
            TrajectoryPlanPoint(
                [position] * 7,
                [0.0] * 7,
                stamp,
                [0.0] * 7,
            )
            for stamp, position in samples
        ],
    )


def _kinematic_cost(result, start, end, *, hold_after_end=False):
    return common_window_kinematic_cost(
        result,
        trajectory_start_unix_s=100.0,
        window_start_unix_s=100.0 + start,
        window_end_unix_s=100.0 + end,
        max_velocity_rad_s=1.5,
        max_acceleration_rad_s2=3.0,
        max_jerk_rad_s3=15.0,
        sample_dt_s=0.02,
        hold_after_end=hold_after_end,
    )


def test_short_old_trajectory_is_compared_as_terminal_hold_instead_of_infinity():
    old = _trajectory([(0.0, 0.0), (1.0, 0.2)])

    assert math.isinf(_kinematic_cost(old, 0.5, 2.5))
    assert math.isfinite(_kinematic_cost(old, 0.5, 2.5, hold_after_end=True))


def test_tail_cost_distinguishes_unnecessary_motion_after_common_window():
    direct = _trajectory([(0.0, 0.0), (1.0, 0.1), (2.0, 0.1), (4.0, 0.1)])
    detour = _trajectory([(0.0, 0.0), (1.0, 0.1), (2.0, 0.8), (4.0, 0.1)])

    assert _kinematic_cost(direct, 0.0, 1.0) == _kinematic_cost(detour, 0.0, 1.0)
    assert _kinematic_cost(direct, 1.0, 4.0) < _kinematic_cost(detour, 1.0, 4.0)
