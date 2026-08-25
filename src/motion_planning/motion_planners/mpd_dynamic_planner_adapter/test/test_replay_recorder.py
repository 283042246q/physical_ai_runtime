import json

import numpy as np

from manipulation_motion_planning.contracts import (
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)
from mpd_dynamic_planner_adapter.dynamic_world import DynamicWorldSnapshot
from mpd_dynamic_planner_adapter.replay_recorder import (
    DynamicReplayRecorder,
    _best_positions_from_archive,
)


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


def test_recorder_preserves_top_k_clearance_diagnostics(tmp_path):
    recorder = _recorder(tmp_path)
    recorder.record_world(
        DynamicWorldSnapshot(1, "fr3_link0", 1_000_000_000, 20_000_000_000, ())
    )
    result = _result()
    result.diagnostics["top_k_clearance_risk"] = [
        {
            "candidate_index": 0,
            "duration_s": 8.5,
            "hard_minimum_clearance_m": 0.08,
            "common_window_minimum_clearance_m": 0.03,
            "clearance_mean_cost": 0.02,
            "clearance_cvar_cost": 0.20,
            "terminal_hold_minimum_clearance_m": 0.03,
        }
    ]
    recorder.record_candidate(
        40,
        result,
        start_unix_s=1.1,
        handoff_unix_s=1.1,
    )

    manifest = json.loads(
        recorder.close(unix_ns=2_000_000_000).read_text(encoding="utf-8")
    )
    assert manifest["plans"][0]["candidate_clearance_diagnostics"] == (
        result.diagnostics["top_k_clearance_risk"]
    )


def test_recorder_reads_schema_v2_best_positions(tmp_path):
    path = tmp_path / "trajectory.npz"
    np.savez(
        path,
        artifact_schema_version=np.asarray(2, dtype=np.int64),
        best_trajectory_topk_index=np.asarray(1, dtype=np.int64),
        topk_positions=np.asarray(
            [
                [[0.0] * 7, [0.1] * 7],
                [[1.0] * 7, [1.1] * 7],
            ]
        ),
    )
    with np.load(path, allow_pickle=False) as data:
        positions = _best_positions_from_archive(data)
    assert positions[0].tolist() == [1.0] * 7


def test_recorder_reads_schema_v3_best_positions(tmp_path):
    path = tmp_path / "trajectory-v3.npz"
    np.savez(
        path,
        artifact_schema_version=np.asarray(3, dtype=np.int64),
        best_trajectory_topk_index=np.asarray(1, dtype=np.int64),
        topk_positions=np.asarray(
            [
                [[0.0] * 7, [0.1] * 7],
                [[1.0] * 7, [1.1] * 7],
            ]
        ),
    )
    with np.load(path, allow_pickle=False) as data:
        positions = _best_positions_from_archive(data)
    assert positions[0].tolist() == [1.0] * 7
