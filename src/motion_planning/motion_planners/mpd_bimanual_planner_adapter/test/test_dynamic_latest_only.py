import json
from types import SimpleNamespace

import pytest

from mpd_bimanual_planner_adapter.collision_guard import CollisionGuard
from mpd_bimanual_planner_adapter.latest_world_buffer import LatestWorldBuffer
from mpd_bimanual_planner_adapter.replan_coordinator import LatestOnlyReplanCoordinator
from mpd_bimanual_planner_adapter.replay_recorder import DynamicReplayRecorder


def _time(ns):
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _object(object_id="box"):
    return SimpleNamespace(
        object_id=object_id,
        SPHERE=0,
        BOX=1,
        CAPSULE=2,
        shape_type=1,
        size_xyz=[1.0, 0.5, 1.5],
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=0.75),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(linear=SimpleNamespace(x=0.1, y=0.0, z=0.0)),
        covariance=[0.0] * 36,
        inflation_m=0.05,
        valid_until=_time(5_000_000_000),
    )


def _world(version=1, frame="world", objects=None):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=_time(1_000_000_000)),
        world_version=version,
        valid_until=_time(4_000_000_000),
        objects=[_object()] if objects is None else objects,
    )


def test_world_conversion_is_fixed_capacity_and_monotonic():
    buffer = LatestWorldBuffer(max_objects=2)
    snapshot = buffer.update(_world(1))
    payload = snapshot.to_worker_payload()
    assert payload["frame_id"] == "world"
    assert payload["objects"][0]["local_sdf"]["type"] == "box"
    assert payload["objects"][0]["linear_velocity"] == [0.1, 0.0, 0.0]
    with pytest.raises(ValueError, match="monotonically"):
        buffer.update(_world(1))
    with pytest.raises(ValueError, match="frame"):
        LatestWorldBuffer().update(_world(1, frame="map"))
    with pytest.raises(ValueError, match="fixed capacity"):
        LatestWorldBuffer(max_objects=1).update(_world(1, objects=[_object("a"), _object("b")]))


def test_new_world_invalidates_completed_old_generation():
    coordinator = LatestOnlyReplanCoordinator(lambda request: request)
    ticket = coordinator.begin(request_seq=4, world_version=10, deadline_unix_ns=1000)
    assert coordinator.accepts(ticket, latest_world_version=10, now_unix_ns=999)
    coordinator.invalidate()
    assert not coordinator.accepts(ticket, latest_world_version=10, now_unix_ns=999)
    newer = coordinator.begin(request_seq=5, world_version=11, deadline_unix_ns=2000)
    assert not coordinator.accepts(newer, latest_world_version=12, now_unix_ns=1500)
    assert not coordinator.accepts(newer, latest_world_version=11, now_unix_ns=2000)


def test_guard_escalates_for_whole_bimanual_goal():
    guard = CollisionGuard(warning_clearance_m=0.05, brake_clearance_m=0.02)
    assert guard.classify(min_clearance_m=0.10) == "safe"
    assert guard.classify(min_clearance_m=0.04) == "replan"
    assert guard.classify(min_clearance_m=0.01) == "brake"
    assert guard.classify(min_clearance_m=1.0, arm_fault=True) == "brake"


def test_dynamic_recorder_writes_deterministic_sequence(tmp_path):
    path = tmp_path / "replay.json"
    recorder = DynamicReplayRecorder(path, request_id="dynamic-demo")
    recorder.record("world", {"world_version": 1}, unix_ns=10)
    recorder.record("handoff", {"mode": "cancel_hold_replan"}, unix_ns=20)
    payload = json.loads(path.read_text())
    assert payload["schema"] == "marvin_bimanual_dynamic_replay/v1"
    assert [event["sequence"] for event in payload["events"]] == [1, 2]
    assert payload["events"][1]["payload"]["mode"] == "cancel_hold_replan"
