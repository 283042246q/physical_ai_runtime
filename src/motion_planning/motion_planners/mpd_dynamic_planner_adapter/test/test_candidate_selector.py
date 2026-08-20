import math

from mpd_dynamic_planner_adapter.candidate_selector import (
    CandidateCost,
    choose_hysteretic_switch,
    clearance_cost,
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


def test_clearance_cost_is_zero_above_preference_and_infinite_at_collision():
    assert clearance_cost(0.2, 0.1) == 0.0
    assert 0.0 < clearance_cost(0.05, 0.1) < 1.0
    assert math.isinf(clearance_cost(0.0, 0.1))
