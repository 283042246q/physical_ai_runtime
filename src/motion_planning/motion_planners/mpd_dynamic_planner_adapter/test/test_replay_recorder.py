import json

from manipulation_motion_planning.contracts import (
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)
from mpd_dynamic_planner_adapter.dynamic_world import DynamicWorldSnapshot
from mpd_dynamic_planner_adapter.replay_recorder import DynamicReplayRecorder


def _result(offset=0.0):
    return TrajectoryPlanResult(
        valid=True,
        joint_names=[f"fr3_joint{index}" for index in range(1, 8)],
        points=[
            TrajectoryPlanPoint([offset] * 7, [0.0] * 7, 0.0),
            TrajectoryPlanPoint([offset + 0.1] * 7, [0.0] * 7, 2.0),
        ],
    )


def _recorder(tmp_path):
    scene = tmp_path / "scene.json"
    scene.write_text(
        json.dumps(
            {
                "schema": "mpd_isaaclab_scene",
                "schema_version": 1,
                "env_name": "EnvOpenDrawerShelf",
                "obstacles": [],
            }
        ),
        encoding="utf-8",
    )
    return DynamicReplayRecorder(
        tmp_path / "episode",
        env_name="EnvOpenDrawerShelf",
        static_scene_path=scene,
    )


def test_recorder_builds_superseded_handoff_timeline(tmp_path):
    recorder = _recorder(tmp_path)
    recorder.record_state(
        StartState([f"fr3_joint{i}" for i in range(1, 8)], [0.0] * 7, [0.0] * 7)
    )
    recorder.record_world(DynamicWorldSnapshot(1, "fr3_link0", 1_000_000_000, 20_000_000_000, ()))
    recorder.record_candidate(
        10,
        _result(),
        start_unix_s=2.0,
        handoff_unix_s=2.0,
    )
    recorder.record_activation(10)
    recorder.record_world(DynamicWorldSnapshot(2, "fr3_link0", 2_000_000_000, 21_000_000_000, ()))
    recorder.record_candidate(
        11,
        _result(0.1),
        start_unix_s=3.0,
        handoff_unix_s=3.0,
    )
    recorder.record_activation(11)

    manifest_path = recorder.close(unix_ns=4_000_000_000)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema"] == "mpd_dynamic_replay"
    assert manifest["env_name"] == "EnvOpenDrawerShelf"
    assert manifest["duration_s"] == 3.0
    assert len(manifest["world_snapshots"]) == 2
    assert [plan["status"] for plan in manifest["plans"]] == ["superseded", "accepted"]
    assert manifest["plans"][0]["active_until_s"] == 2.0
    assert [event["type"] for event in manifest["events"]] == ["handoff", "handoff"]
    assert (manifest_path.parent / manifest["plans"][0]["trajectory"]).is_file()


def test_recorder_marks_braking_plan_and_event(tmp_path):
    recorder = _recorder(tmp_path)
    recorder.record_world(DynamicWorldSnapshot(1, "fr3_link0", 1_000_000_000, 20_000_000_000, ()))
    recorder.record_candidate(
        20,
        _result(),
        start_unix_s=1.1,
        handoff_unix_s=None,
        braking=True,
        reason="guard_collision",
    )
    recorder.record_activation(20)
    manifest = json.loads(recorder.close(unix_ns=2_000_000_000).read_text(encoding="utf-8"))
    assert manifest["plans"][0]["status"] == "braking"
    assert manifest["events"][0]["type"] == "brake"
    assert manifest["events"][0]["reason"] == "guard_collision"


def test_recorder_flush_checkpoints_before_shutdown(tmp_path):
    recorder = _recorder(tmp_path)
    recorder.record_world(
        DynamicWorldSnapshot(1, "fr3_link0", 1_000_000_000, 20_000_000_000, ())
    )
    recorder.record_candidate(
        30,
        _result(),
        start_unix_s=1.1,
        handoff_unix_s=1.1,
    )
    recorder.record_activation(30)
    checkpoint = recorder.flush(unix_ns=1_500_000_000)

    assert checkpoint is not None
    manifest = json.loads(checkpoint.read_text(encoding="utf-8"))
    assert manifest["duration_s"] == 0.5
    assert manifest["plans"][0]["status"] == "accepted"
    assert manifest["plans"][0]["active_until_s"] == 0.5
