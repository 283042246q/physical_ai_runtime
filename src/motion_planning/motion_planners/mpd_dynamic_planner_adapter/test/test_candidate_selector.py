import math

from manipulation_motion_planning.contracts import (
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)
from mpd_dynamic_planner_adapter.candidate_selector import (
    CandidateCost,
    adaptive_deviation_weight,
    choose_hysteretic_switch,
    clearance_cost,
    common_window_deviation_cost,
    common_window_kinematic_cost,
)


def _adaptive_weight(clearance, collision_time=None, *, hard_safe=True):
    return adaptive_deviation_weight(
        0.15,
        minimum_clearance_m=clearance,
        first_collision_unix_s=collision_time,
        reference_unix_s=100.0,
        old_hard_safe=hard_safe,
        clearance_zero_m=0.02,
        clearance_full_m=0.10,
        ttc_zero_s=2.0,
        ttc_full_s=5.0,
    )


def test_adaptive_deviation_keeps_full_weight_for_safe_distant_old_plan():
    schedule = _adaptive_weight(0.12)

    assert schedule.effective_weight == 0.15
    assert schedule.clearance_gate == 1.0
    assert schedule.ttc_gate == 1.0
    assert math.isinf(schedule.predicted_ttc_s)


def test_adaptive_deviation_uses_the_more_conservative_smooth_risk_gate():
    clearance_limited = _adaptive_weight(0.06, 110.0)
    ttc_limited = _adaptive_weight(0.12, 103.5)

    assert math.isclose(clearance_limited.clearance_gate, 0.5)
    assert math.isclose(clearance_limited.effective_weight, 0.075)
    assert math.isclose(ttc_limited.ttc_gate, 0.5)
    assert math.isclose(ttc_limited.effective_weight, 0.075)


def test_adaptive_deviation_is_zero_for_imminent_or_hard_unsafe_old_plan():
    imminent = _adaptive_weight(0.12, 101.5)
    hard_unsafe = _adaptive_weight(0.12, 110.0, hard_safe=False)

    assert imminent.effective_weight == 0.0
    assert hard_unsafe.effective_weight == 0.0


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


def test_relative_hysteresis_scales_with_safe_old_cost_and_is_strict():
    rejected = choose_hysteretic_switch(
        [_candidate(0, 9.0)],
        old_cost=10.0,
        old_safe=True,
        minimum_commit_interval_elapsed=True,
        switching_hysteresis=0.02,
        relative_hysteresis=0.10,
    )
    accepted = choose_hysteretic_switch(
        [_candidate(0, 8.99)],
        old_cost=10.0,
        old_safe=True,
        minimum_commit_interval_elapsed=True,
        switching_hysteresis=0.02,
        relative_hysteresis=0.10,
    )

    assert rejected.candidate_index is None
    assert rejected.reason == "switching_hysteresis"
    assert accepted.candidate_index == 0


def test_phase4_absolute_hysteresis_preserves_threshold_equality_switch():
    decision = choose_hysteretic_switch(
        [_candidate(0, 0.25)],
        old_cost=0.5,
        old_safe=True,
        minimum_commit_interval_elapsed=True,
        switching_hysteresis=0.25,
        relative_hysteresis=0.0,
    )

    assert decision.candidate_index == 0


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


def test_common_window_deviation_is_time_aligned_and_holds_short_trajectory():
    old = _trajectory([(0.0, 0.0), (1.0, 0.0)])
    same = _trajectory([(0.0, 0.0), (2.0, 0.0)])
    changed = _trajectory([(0.0, 0.0), (1.0, 0.5), (2.0, 0.5)])

    def deviation(new):
        return common_window_deviation_cost(
            new,
            old,
            new_trajectory_start_unix_s=100.0,
            old_trajectory_start_unix_s=100.0,
            window_start_unix_s=100.0,
            window_end_unix_s=102.0,
            sample_dt_s=0.02,
            hold_after_end=True,
        )

    assert deviation(same) == 0.0
    assert deviation(changed) > 0.0
