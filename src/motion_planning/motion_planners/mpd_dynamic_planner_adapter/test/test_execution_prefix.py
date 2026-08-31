import pytest

from manipulation_motion_planning.contracts import (
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)
from mpd_planner_adapter.trajectory import TimedPlan

from mpd_dynamic_planner_adapter.replan_node import _prepend_execution_prefix


NAMES = [f"fr3_joint{index}" for index in range(1, 8)]


def _point(value, velocity, time_from_start):
    return TrajectoryPlanPoint(
        positions=[value] * 7,
        velocities=[velocity] * 7,
        accelerations=[0.0] * 7,
        time_from_start_s=time_from_start,
    )


def test_explicit_execution_prefix_contains_old_motion_hold_bridge_and_suffix():
    active = TimedPlan(
        TrajectoryPlanResult(
            valid=True,
            joint_names=NAMES,
            points=[_point(0.0, 0.2, 0.0), _point(0.2, 0.0, 2.0)],
        ),
        10.0,
    )
    selected = TrajectoryPlanResult(
        valid=True,
        joint_names=NAMES,
        points=[
            _point(0.2, 0.0, 0.0),
            _point(0.3, 0.1, 0.5),
            _point(0.5, 0.0, 1.5),
        ],
    )

    command = _prepend_execution_prefix(
        active,
        selected,
        monitoring_start_unix_s=11.0,
        bridge_start_unix_s=13.0,
        sample_dt_s=1.0,
    )

    assert [point.time_from_start_s for point in command.points] == pytest.approx(
        [0.0, 1.0, 2.0, 2.5, 3.5]
    )
    assert command.points[0].positions == pytest.approx([0.1] * 7)
    assert command.points[1].positions == pytest.approx([0.2] * 7)
    assert command.points[2].positions == pytest.approx([0.2] * 7)
    assert command.points[2].velocities == pytest.approx([0.0] * 7)
    assert command.points[-1].positions == pytest.approx([0.5] * 7)


def test_first_plan_prefix_explicitly_holds_the_selected_start_state():
    selected = TrajectoryPlanResult(
        valid=True,
        joint_names=NAMES,
        points=[_point(0.4, 0.0, 0.0), _point(0.5, 0.0, 1.0)],
    )

    command = _prepend_execution_prefix(
        None,
        selected,
        monitoring_start_unix_s=4.0,
        bridge_start_unix_s=5.0,
        sample_dt_s=0.5,
    )

    assert [point.time_from_start_s for point in command.points] == pytest.approx(
        [0.0, 0.5, 1.0, 2.0]
    )
    assert all(
        point.positions == pytest.approx([0.4] * 7)
        for point in command.points[:3]
    )
