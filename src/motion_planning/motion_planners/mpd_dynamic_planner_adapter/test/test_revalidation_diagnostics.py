from types import SimpleNamespace

import numpy as np
import pytest

from mpd_dynamic_planner_adapter.collision_guard import (
    DynamicTrajectoryGuard,
    TimedCollisionPlan,
)
from mpd_dynamic_planner_adapter.dynamic_world import DynamicWorldSnapshot
from mpd_dynamic_planner_adapter.replan_node import (
    CANDIDATE_REJECTION_REASONS,
    MpdDynamicReplanNode,
    _risk_rejection_reason,
    _validation_start_unix_s,
)


def test_validation_time_is_sampled_inside_selected_world_epoch():
    world = DynamicWorldSnapshot(
        version=7,
        frame_id="fr3_link0",
        stamp_unix_ns=10_000_000_000,
        valid_until_unix_ns=20_000_000_000,
        objects=(),
    )
    plan = TimedCollisionPlan(
        absolute_times_s=np.asarray([9.9, 11.0]),
        sphere_positions=np.zeros((2, 1, 3)),
        sphere_radii=np.asarray([0.1]),
    )
    guard = DynamicTrajectoryGuard()

    stale_risk = guard.validate(plan, world, 9.9, 11.0)
    validation_now = _validation_start_unix_s(world, now_unix_s=9.9)
    current_risk = guard.validate(plan, world, validation_now, 11.0)

    assert not stale_risk.safe
    assert stale_risk.checked_samples == 0
    assert validation_now > 10.0
    assert validation_now * 1e9 >= world.stamp_unix_ns
    assert current_risk.safe
    assert current_risk.checked_samples > 0


@pytest.mark.parametrize(
    ("checked_samples", "expected"),
    [(0, "invalid_time_interval"), (1, "dynamic_collision")],
)
def test_guard_rejection_reason_distinguishes_time_from_collision(
    checked_samples, expected
):
    risk = SimpleNamespace(checked_samples=checked_samples)
    assert _risk_rejection_reason(risk) == expected


def test_candidate_rejections_preserve_candidate_attempt_stage_and_reason():
    holder = SimpleNamespace(_last_candidate_rejections=[])
    for index, reason in enumerate(CANDIDATE_REJECTION_REASONS):
        MpdDynamicReplanNode._record_candidate_rejection(
            holder,
            index,
            reason,
            attempt=1,
            stage="test_stage",
        )

    assert holder._last_candidate_rejections == [
        {
            "candidate_index": index,
            "attempt": 2,
            "stage": "test_stage",
            "reason": reason,
        }
        for index, reason in enumerate(CANDIDATE_REJECTION_REASONS)
    ]
