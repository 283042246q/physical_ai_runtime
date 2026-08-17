import numpy as np
import pytest

from mpd_dynamic_planner_adapter.collision_guard import (
    DynamicTrajectoryGuard,
    TimedCollisionPlan,
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
