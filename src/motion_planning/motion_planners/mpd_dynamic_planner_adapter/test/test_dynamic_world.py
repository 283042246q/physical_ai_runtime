import numpy as np
import pytest

import mpd_dynamic_planner_adapter.dynamic_world as dynamic_world_module
from mpd_dynamic_planner_adapter.dynamic_world import (
    DynamicWorldError,
    DynamicWorldManager,
)


def _observation(stamp, x, **item_overrides):
    item = {
        "id": "box",
        "local_sdf": {"type": "box", "size_xyz": [0.2, 0.3, 0.4]},
        "position": [x, 0.0, 0.0],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "position_covariance_3x3": (np.eye(3) * 1e-4).reshape(-1).tolist(),
    }
    item.update(item_overrides)
    return {"frame_id": "fr3_link0", "stamp_unix_ns": stamp, "objects": [item]}


def test_constant_velocity_filter_and_worker_snapshot_contract():
    manager = DynamicWorldManager(
        prediction_horizon_s=12.0,
        initial_velocity_std_m_s=2.0,
        process_acceleration_std_m_s2=0.01,
        initial_version=0,
    )
    first = manager.update(_observation(1_000_000_000, 0.0))
    second = manager.update(_observation(2_000_000_000, 1.0))
    assert first.version == 1
    assert second.version == 2
    assert second.objects[0].linear_velocity[0] > 0.9
    worker = second.to_worker_dict()
    assert worker["world_version"] == 2
    assert worker["valid_until_unix_ns"] == 14_000_000_000
    assert worker["objects"][0]["local_sdf"]["type"] == "box"
    assert len(worker["objects"][0]["covariance_6x6"]) == 36


def test_default_version_namespace_survives_ros_node_restart(monkeypatch):
    monkeypatch.setattr(dynamic_world_module.time, "time_ns", lambda: 123_000)
    manager = DynamicWorldManager()
    snapshot = manager.update(_observation(1_000_000_000, 0.0))
    assert snapshot.version == 123_001


def test_orientation_updates_but_future_model_has_no_angular_velocity():
    manager = DynamicWorldManager(initial_version=0)
    manager.update(_observation(1_000_000_000, 0.0))
    snapshot = manager.update(
        _observation(
            2_000_000_000,
            0.0,
            orientation_xyzw=[0.0, 0.0, 2**-0.5, 2**-0.5],
        )
    )
    assert snapshot.objects[0].orientation_xyzw == pytest.approx(
        (0.0, 0.0, 2**-0.5, 2**-0.5)
    )
    assert "angular_velocity" not in snapshot.objects[0].to_worker_dict()


def test_local_sdf_identity_cannot_change_and_time_cannot_reverse():
    manager = DynamicWorldManager(initial_version=0)
    manager.update(_observation(2_000_000_000, 0.0))
    with pytest.raises(DynamicWorldError, match="local_sdf"):
        manager.update(
            _observation(
                3_000_000_000,
                0.0,
                local_sdf={"type": "sphere", "radius": 0.1},
            )
        )
    with pytest.raises(DynamicWorldError, match="backwards"):
        manager.update(_observation(1_000_000_000, 0.0))
