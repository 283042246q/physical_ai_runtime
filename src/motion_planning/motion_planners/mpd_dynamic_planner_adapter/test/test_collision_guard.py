import numpy as np
import pytest

from mpd_dynamic_planner_adapter.collision_guard import (
    DynamicTrajectoryGuard,
    TimedCollisionPlan,
    extend_collision_plan_with_terminal_hold,
    validate_collision_plan_actual_duration,
)
from mpd_dynamic_planner_adapter.dynamic_world import (
    DynamicObjectSnapshot,
    DynamicWorldSnapshot,
)


def _object(position=(0.0, 0.0, 0.0), velocity=(1.0, 0.0, 0.0)):
    return DynamicObjectSnapshot(
        object_id="sphere",
        local_sdf={"type": "sphere", "radius": 0.1},
        position=position,
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        linear_velocity=velocity,
        covariance_6x6=tuple(np.zeros((6, 6)).reshape(-1)),
        inflation_mode="linear",
        base_inflation_m=0.0,
        horizon_inflation_rate_m_s=0.0,
    )


def _world(item):
    return DynamicWorldSnapshot(
        version=3,
        frame_id="fr3_link0",
        stamp_unix_ns=0,
        valid_until_unix_ns=3_000_000_000,
        objects=(item,),
    )


def test_old_trajectory_and_obstacle_are_compared_at_same_absolute_time():
    plan = TimedCollisionPlan(
        absolute_times_s=np.asarray([0.0, 1.0, 2.0]),
        sphere_positions=np.asarray([[[1.0, 0.0, 0.0]]] * 3),
        sphere_radii=np.asarray([0.05]),
    )
    risk = DynamicTrajectoryGuard(check_dt_s=0.01).validate(plan, _world(_object()), 0.0, 2.0)
    assert not risk.safe
    assert risk.world_version == 3
    assert risk.first_collision_unix_s == pytest.approx(0.85, abs=0.02)
    assert risk.minimum_clearance_m == pytest.approx(-0.15, abs=1e-6)


def test_orientation_is_used_for_local_box_sdf():
    item = _object(position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0))
    item = DynamicObjectSnapshot(
        **{
            **item.__dict__,
            "local_sdf": {"type": "box", "size_xyz": [2.0, 0.2, 0.2]},
            "orientation_xyzw": (0.0, 0.0, 2**-0.5, 2**-0.5),
        }
    )
    plan = TimedCollisionPlan(
        absolute_times_s=np.asarray([0.0, 1.0]),
        sphere_positions=np.asarray([[[0.0, 0.8, 0.0]], [[0.0, 0.8, 0.0]]]),
        sphere_radii=np.asarray([0.05]),
    )
    risk = DynamicTrajectoryGuard().validate(plan, _world(item), 0.0, 1.0)
    assert not risk.safe
    assert risk.minimum_clearance_m == pytest.approx(-0.15)


def test_prediction_outside_validity_fails_closed():
    plan = TimedCollisionPlan(
        absolute_times_s=np.asarray([0.0, 4.0]),
        sphere_positions=np.zeros((2, 1, 3)),
        sphere_radii=np.asarray([0.01]),
    )
    risk = DynamicTrajectoryGuard().validate(plan, _world(_object()), 0.0, 4.0)
    assert not risk.safe
    assert risk.checked_samples == 0


def test_terminal_hold_extension_keeps_last_robot_occupancy_for_future_checks():
    plan = TimedCollisionPlan(
        absolute_times_s=np.asarray([0.0, 1.0]),
        sphere_positions=np.asarray([[[0.0, 0.0, 0.0]], [[1.0, 0.0, 0.0]]]),
        sphere_radii=np.asarray([0.05]),
    )

    held = extend_collision_plan_with_terminal_hold(plan, 3.0)

    assert held.absolute_times_s[-1] == 3.0
    np.testing.assert_allclose(held.sample(np.asarray([1.5, 2.5])), [[[1.0, 0.0, 0.0]]] * 2)
    risk = DynamicTrajectoryGuard(check_dt_s=0.01).validate(
        held,
        _world(_object(position=(3.0, 0.0, 0.0), velocity=(-1.0, 0.0, 0.0))),
        1.0,
        3.0,
    )
    assert not risk.safe
    assert risk.first_collision_unix_s == pytest.approx(1.85, abs=0.02)


def test_candidate_hard_collision_check_stops_at_actual_duration():
    plan = TimedCollisionPlan(
        absolute_times_s=np.asarray([0.0, 1.0]),
        sphere_positions=np.asarray([[[0.0, 0.0, 0.0]], [[1.0, 0.0, 0.0]]]),
        sphere_radii=np.asarray([0.05]),
    )
    world = _world(_object(position=(3.0, 0.0, 0.0), velocity=(-1.0, 0.0, 0.0)))

    actual = validate_collision_plan_actual_duration(
        DynamicTrajectoryGuard(check_dt_s=0.01), plan, world, 0.0
    )
    held = extend_collision_plan_with_terminal_hold(plan, 3.0)
    fixed_horizon = DynamicTrajectoryGuard(check_dt_s=0.01).validate(
        held, world, 0.0, 3.0
    )

    assert actual.safe
    assert actual.checked_samples == 101
    assert not fixed_horizon.safe
    assert fixed_horizon.first_collision_unix_s == pytest.approx(1.85, abs=0.02)


def test_common_window_clearance_statistics_include_terminal_hold():
    plan = TimedCollisionPlan(
        absolute_times_s=np.asarray([0.0, 1.0]),
        sphere_positions=np.asarray([[[0.0, 0.0, 0.0]], [[1.0, 0.0, 0.0]]]),
        sphere_radii=np.asarray([0.05]),
    )
    world = _world(_object(position=(3.0, 0.0, 0.0), velocity=(-1.0, 0.0, 0.0)))
    guard = DynamicTrajectoryGuard(check_dt_s=0.01)

    hard_risk = validate_collision_plan_actual_duration(guard, plan, world, 0.0)
    score_risk = guard.validate(
        extend_collision_plan_with_terminal_hold(plan, 3.0),
        world,
        0.0,
        3.0,
        preferred_clearance_m=0.10,
        cvar_fraction=0.10,
        terminal_hold_start_unix_s=1.0,
    )

    assert hard_risk.safe
    assert not score_risk.safe
    assert score_risk.minimum_clearance_m < hard_risk.minimum_clearance_m
    assert score_risk.clearance_mean_cost > 0.0
    assert score_risk.clearance_cvar_cost >= score_risk.clearance_mean_cost
    assert score_risk.terminal_hold_minimum_clearance_m == pytest.approx(
        score_risk.minimum_clearance_m
    )


def test_clearance_profile_parameters_fail_closed():
    plan = TimedCollisionPlan(
        absolute_times_s=np.asarray([0.0, 1.0]),
        sphere_positions=np.zeros((2, 1, 3)),
        sphere_radii=np.asarray([0.01]),
    )
    guard = DynamicTrajectoryGuard()

    with pytest.raises(ValueError, match="preferred_clearance_m"):
        guard.validate(plan, _world(_object()), 0.0, 1.0, preferred_clearance_m=0.0)
    with pytest.raises(ValueError, match="cvar_fraction"):
        guard.validate(
            plan,
            _world(_object()),
            0.0,
            1.0,
            preferred_clearance_m=0.1,
            cvar_fraction=0.0,
        )


def test_common_window_score_is_independent_of_original_plan_duration():
    short = TimedCollisionPlan(
        absolute_times_s=np.asarray([0.0, 1.0]),
        sphere_positions=np.asarray([[[0.0, 0.0, 0.0]], [[1.0, 0.0, 0.0]]]),
        sphere_radii=np.asarray([0.05]),
    )
    long = TimedCollisionPlan(
        absolute_times_s=np.asarray([0.0, 1.0, 3.0]),
        sphere_positions=np.asarray(
            [[[0.0, 0.0, 0.0]], [[1.0, 0.0, 0.0]], [[1.0, 0.0, 0.0]]]
        ),
        sphere_radii=np.asarray([0.05]),
    )
    world = _world(_object(position=(3.0, 0.0, 0.0), velocity=(-1.0, 0.0, 0.0)))
    guard = DynamicTrajectoryGuard(check_dt_s=0.01)

    short_score = guard.validate(
        extend_collision_plan_with_terminal_hold(short, 3.0),
        world,
        0.0,
        3.0,
        preferred_clearance_m=0.10,
        cvar_fraction=0.10,
        terminal_hold_start_unix_s=1.0,
    )
    long_score = guard.validate(
        long,
        world,
        0.0,
        3.0,
        preferred_clearance_m=0.10,
        cvar_fraction=0.10,
        terminal_hold_start_unix_s=3.0,
    )

    assert short_score.minimum_clearance_m == pytest.approx(
        long_score.minimum_clearance_m
    )
    assert short_score.clearance_mean_cost == pytest.approx(
        long_score.clearance_mean_cost
    )
    assert short_score.clearance_cvar_cost == pytest.approx(
        long_score.clearance_cvar_cost
    )
