import numpy as np
import pytest

from manipulation_motion_planning.contracts import (
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)
from mpd_planner_adapter.trajectory import TimedPlan

from mpd_dynamic_planner_adapter.braking import make_braking_plan
from mpd_dynamic_planner_adapter.collision_guard import (
    DynamicTrajectoryGuard,
    TimedCollisionPlan,
)
from mpd_dynamic_planner_adapter.dynamic_world import DynamicWorldSnapshot
from mpd_dynamic_planner_adapter.handoff_selector import (
    select_earliest_low_speed_handoff,
)


NAMES = [f"fr3_joint{i}" for i in range(1, 8)]


def _timed_plan():
    result = TrajectoryPlanResult(
        valid=True,
        joint_names=NAMES,
        points=[
            TrajectoryPlanPoint([0.0] * 7, [0.4] * 7, 0.0),
            TrajectoryPlanPoint([0.2] * 7, [0.1] * 7, 1.0),
            TrajectoryPlanPoint([0.3] * 7, [0.0] * 7, 2.0),
        ],
    )
    return TimedPlan(result, 10.0)


def test_selects_earliest_dynamic_safe_low_speed_time():
    active = _timed_plan()
    collision = TimedCollisionPlan(
        absolute_times_s=np.asarray([10.0, 11.0, 12.0]),
        sphere_positions=np.zeros((3, 1, 3)),
        sphere_radii=np.asarray([0.05]),
    )
    world = DynamicWorldSnapshot(1, "fr3_link0", 10_000_000_000, 13_000_000_000, ())
    choice = select_earliest_low_speed_handoff(
        active_plan=active,
        collision_plan=collision,
        world=world,
        guard=DynamicTrajectoryGuard(),
        now_unix_s=10.0,
        earliest_unix_s=10.5,
        latest_unix_s=12.0,
        step_s=0.1,
        max_speed_rad_s=0.2,
    )
    assert choice.handoff_unix_s == pytest.approx(10.7)
    assert choice.reason == "earliest_dynamic_safe_low_speed"


def test_braking_plan_reaches_zero_velocity_continuously():
    state = StartState(NAMES, [0.0] * 7, [0.4, -0.2, 0.0, 0.1, 0.0, 0.0, 0.0], 1.0)
    plan = make_braking_plan(state, max_deceleration_rad_s2=1.0, sample_dt_s=0.05)
    assert plan.valid
    assert plan.points[0].positions == pytest.approx(state.positions)
    assert plan.points[0].velocities == pytest.approx(state.velocities)
    assert plan.points[-1].velocities == pytest.approx([0.0] * 7)
    assert plan.diagnostics["duration_s"] == pytest.approx(0.4)
    assert max(
        right.time_from_start_s - left.time_from_start_s
        for left, right in zip(plan.points, plan.points[1:])
    ) <= 0.0500001


def test_no_low_speed_candidate_returns_no_handoff():
    result = TrajectoryPlanResult(
        valid=True,
        joint_names=NAMES,
        points=[
            TrajectoryPlanPoint([0.0] * 7, [0.4] * 7, 0.0),
            TrajectoryPlanPoint([0.2] * 7, [0.4] * 7, 1.0),
            TrajectoryPlanPoint([0.4] * 7, [0.4] * 7, 2.0),
        ],
    )
    active = TimedPlan(result, 10.0)
    collision = TimedCollisionPlan(
        absolute_times_s=np.asarray([10.0, 11.0, 12.0]),
        sphere_positions=np.zeros((3, 1, 3)),
        sphere_radii=np.asarray([0.05]),
    )
    world = DynamicWorldSnapshot(1, "fr3_link0", 10_000_000_000, 13_000_000_000, ())
    choice = select_earliest_low_speed_handoff(
        active_plan=active,
        collision_plan=collision,
        world=world,
        guard=DynamicTrajectoryGuard(),
        now_unix_s=10.0,
        earliest_unix_s=10.5,
        latest_unix_s=12.0,
        max_speed_rad_s=0.2,
    )
    assert choice.handoff_unix_s is None
    assert choice.reason == "no_dynamic_safe_low_speed_handoff"
