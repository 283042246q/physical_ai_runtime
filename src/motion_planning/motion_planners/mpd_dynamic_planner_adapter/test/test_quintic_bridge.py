import numpy as np
import pytest

from manipulation_motion_planning.contracts import (
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)
from mpd_dynamic_planner_adapter.quintic_bridge import (
    minimum_bridge_duration,
    sample_quintic,
    splice_with_quintic_bridge,
)


def _boundary(distance):
    zeros = np.zeros(7)
    return zeros, zeros, zeros, np.full(7, distance), zeros, zeros


def test_quintic_exactly_matches_q_dq_ddq_boundaries():
    q0 = np.arange(7) * 0.01
    dq0 = np.arange(7) * 0.02
    ddq0 = np.arange(7) * -0.01
    q1 = q0 + 0.2
    dq1 = dq0 - 0.03
    ddq1 = ddq0 + 0.04

    _, q, dq, ddq, _ = sample_quintic(
        q0, dq0, ddq0, q1, dq1, ddq1, 0.8, sample_dt_s=0.01
    )

    np.testing.assert_allclose(q[[0, -1]], [q0, q1], atol=1e-10)
    np.testing.assert_allclose(dq[[0, -1]], [dq0, dq1], atol=1e-10)
    np.testing.assert_allclose(ddq[[0, -1]], [ddq0, ddq1], atol=1e-9)


def test_minimum_duration_increases_with_position_change():
    limits = dict(
        minimum_duration_s=0.1,
        maximum_duration_s=3.0,
        max_velocity_rad_s=1.0,
        max_acceleration_rad_s2=2.0,
        max_jerk_rad_s3=10.0,
        sample_dt_s=0.005,
    )
    short, _ = minimum_bridge_duration(*_boundary(0.05), **limits)
    long, _ = minimum_bridge_duration(*_boundary(0.5), **limits)

    assert long > short


def test_splice_contains_continuous_quintic_then_complete_mpd_suffix():
    new = TrajectoryPlanResult(
        valid=True,
        joint_names=[f"fr3_joint{i}" for i in range(1, 8)],
        points=[
            TrajectoryPlanPoint([0.2] * 7, [0.1] * 7, 0.0, [0.05] * 7),
            TrajectoryPlanPoint([0.3] * 7, [0.0] * 7, 1.0, [0.0] * 7),
        ],
    )
    current = StartState(list(new.joint_names), [0.0] * 7, [0.0] * 7, 1.0)

    merged = splice_with_quintic_bridge(
        current_state=current,
        current_acceleration=[0.0] * 7,
        new_plan=new,
        duration_s=1.0,
        sample_dt_s=0.05,
        max_velocity_rad_s=2.0,
        max_acceleration_rad_s2=4.0,
        max_jerk_rad_s3=20.0,
    )

    first, handoff, final = merged.points[0], merged.points[-2], merged.points[-1]
    assert first.positions == pytest.approx(current.positions)
    assert handoff.positions == pytest.approx(new.points[0].positions)
    assert handoff.velocities == pytest.approx(new.points[0].velocities)
    assert handoff.accelerations == pytest.approx(new.points[0].accelerations)
    assert final.positions == new.points[1].positions
    assert final.time_from_start_s == pytest.approx(2.0)
