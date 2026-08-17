import pytest

from manipulation_motion_planning.contracts import (
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)
from mpd_planner_adapter.handoff import HandoffValidationError, splice_for_handoff
from mpd_planner_adapter.trajectory import TimedPlan


JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]


def _plan(q0=0.0, q1=0.1, v0=0.0, v1=0.0):
    return TrajectoryPlanResult(
        valid=True,
        joint_names=JOINTS,
        points=[
            TrajectoryPlanPoint([q0] * 7, [v0] * 7, 0.0),
            TrajectoryPlanPoint([q1] * 7, [v1] * 7, 1.0),
        ],
    )


def test_stopped_first_handoff_builds_hold_prefix_and_new_suffix():
    current = StartState(JOINTS, [0.0] * 7, [0.0] * 7)
    merged = splice_for_handoff(
        current_state=current,
        active_plan=None,
        new_plan=_plan(),
        commit_start_unix_s=10.0,
        handoff_unix_s=10.8,
    )
    assert merged.valid
    assert merged.points[0].time_from_start_s == 0.0
    assert merged.points[1].time_from_start_s == pytest.approx(0.8)
    assert merged.points[-1].time_from_start_s == pytest.approx(1.8)
    assert merged.diagnostics["splice_q_jump_rad"] == 0.0


def test_position_jump_is_rejected():
    current = StartState(JOINTS, [0.0] * 7, [0.0] * 7)
    with pytest.raises(HandoffValidationError, match="q jump"):
        splice_for_handoff(
            current_state=current,
            active_plan=None,
            new_plan=_plan(q0=0.2),
            commit_start_unix_s=10.0,
            handoff_unix_s=10.8,
        )


def test_active_plan_start_drift_is_rejected():
    current = StartState(JOINTS, [0.3] * 7, [0.0] * 7)
    active = TimedPlan(_plan(q0=0.0, q1=0.0), 10.0)
    with pytest.raises(HandoffValidationError, match="start drift"):
        splice_for_handoff(
            current_state=current,
            active_plan=active,
            new_plan=_plan(q0=0.0),
            commit_start_unix_s=10.1,
            handoff_unix_s=10.8,
        )


def test_low_speed_gate_rejects_moving_handoff():
    current = StartState(JOINTS, [0.0] * 7, [0.4] * 7)
    with pytest.raises(HandoffValidationError, match="speed"):
        splice_for_handoff(
            current_state=current,
            active_plan=None,
            new_plan=_plan(v0=0.4, v1=0.4),
            commit_start_unix_s=10.0,
            handoff_unix_s=10.8,
        )
