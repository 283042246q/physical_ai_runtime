import time

import numpy as np

from manipulation_motion_planning.contracts import JointTarget, StartState

from mpd_dynamic_planner_adapter.backend import (
    DynamicMpdGlobalTrajectoryBackend,
    SpaceTimeMpdGlobalTrajectoryBackend,
)
from mpd_dynamic_planner_adapter.dynamic_world import DynamicWorldSnapshot


NAMES = [f"fr3_joint{i}" for i in range(1, 8)]


class FakeClient:
    def __init__(self, trajectory_path):
        self.trajectory_path = trajectory_path
        self.updated = None

    def health(self):
        return {
            "status": "OK",
            "state": "READY",
            "engine": {
                "dense_validation": {
                    "fully_warmed": True,
                    "full_batch": True,
                    "pruning_used": False,
                }
            },
        }

    def update_world(self, snapshot):
        self.updated = snapshot
        return {"status": "OK", "world_version": snapshot.version}

    def plan_dynamic(self, request, **options):
        assert options["trajectory_start_unix_ns"] == 2_000_000_000
        return {
            "status": "OK",
            "request_seq": options["request_seq"],
            "world_version": options["world_version"],
            "trajectory_path": str(self.trajectory_path),
            "elapsed_sec": 0.5,
            "engine_instance_id": "dynamic",
        }


class SpaceTimeFakeClient(FakeClient):
    def health(self):
        response = super().health()
        response["engine"]["space_time"] = {
            "enabled": True,
            "mode": "phase5_joint",
            "timing_schema_version": 1,
            "trajectory_schema_version": 3,
            "candidate_specific_time": True,
        }
        return response


def test_dynamic_backend_uploads_exact_version_and_reads_collision_spheres(tmp_path):
    path = tmp_path / "trajectory.npz"
    np.savez(
        path,
        positions=np.zeros((2, 7)),
        velocities=np.zeros((2, 7)),
        accelerations=np.zeros((2, 7)),
        time_from_start=np.asarray([0.0, 10.0]),
        joint_names=np.asarray(NAMES),
        collision_sphere_positions=np.zeros((2, 56, 3)),
        collision_sphere_radii=np.full(56, 0.01),
        topk_positions=np.zeros((2, 2, 7)),
        topk_velocities=np.zeros((2, 2, 7)),
        topk_accelerations=np.zeros((2, 2, 7)),
        topk_scores=np.asarray([0.1, 0.2]),
        topk_source_candidate_indices=np.asarray([3, 7]),
        topk_collision_sphere_positions=np.zeros((2, 2, 56, 3)),
    )
    backend = DynamicMpdGlobalTrajectoryBackend("/tmp/not-used.sock")
    backend.client = FakeClient(path)
    backend.warmup()
    snapshot = DynamicWorldSnapshot(
        4,
        "fr3_link0",
        1_000_000_000,
        20_000_000_000,
        (),
    )
    backend.update_world_snapshot(snapshot)
    result = backend.plan(
        StartState(NAMES, [0.0] * 7, [0.0] * 7, 1.0),
        JointTarget(NAMES, [0.1] * 7),
        {
            "request_seq": 5,
            "world_version": 4,
            "handoff_unix_ns": 2_000_000_000,
            "deadline_unix_ns": time.time_ns() + 1_000_000_000,
        },
    )
    assert result.valid
    assert result.diagnostics["world_version"] == 4
    assert result.diagnostics["collision_sphere_positions"].shape == (2, 56, 3)
    assert result.diagnostics["top_k_count"] == 2
    assert len(result.diagnostics["top_k_candidates"]) == 2
    assert result.points[0].accelerations == [0.0] * 7
    assert backend.client.updated is snapshot


def test_dynamic_backend_rejects_unuploaded_world_version():
    backend = DynamicMpdGlobalTrajectoryBackend("/tmp/not-used.sock")
    result = backend.plan(
        StartState(NAMES, [0.0] * 7, [0.0] * 7, 1.0),
        JointTarget(NAMES, [0.1] * 7),
        {
            "request_seq": 5,
            "world_version": 9,
            "handoff_unix_ns": 2_000_000_000,
            "deadline_unix_ns": None,
        },
    )
    assert not result.valid
    assert "not uploaded" in result.reason


def test_dynamic_backend_reads_deduplicated_schema_v2_float32_spheres(tmp_path):
    path = tmp_path / "trajectory-v2.npz"
    np.savez(
        path,
        artifact_schema_version=np.asarray(2, dtype=np.int64),
        best_trajectory_topk_index=np.asarray(0, dtype=np.int64),
        time_from_start=np.asarray([0.0, 10.0]),
        joint_names=np.asarray(NAMES),
        collision_sphere_radii=np.full(56, 0.01, dtype=np.float32),
        topk_positions=np.zeros((2, 2, 7)),
        topk_velocities=np.zeros((2, 2, 7)),
        topk_accelerations=np.zeros((2, 2, 7)),
        topk_scores=np.asarray([0.1, 0.2]),
        topk_source_candidate_indices=np.asarray([3, 7]),
        topk_collision_sphere_positions=np.zeros((2, 2, 56, 3), dtype=np.float32),
    )
    backend = DynamicMpdGlobalTrajectoryBackend("/tmp/not-used.sock")
    backend.client = FakeClient(path)
    backend.warmup()
    snapshot = DynamicWorldSnapshot(
        4, "fr3_link0", 1_000_000_000, 20_000_000_000, ()
    )
    backend.update_world_snapshot(snapshot)
    result = backend.plan(
        StartState(NAMES, [0.0] * 7, [0.0] * 7, 1.0),
        JointTarget(NAMES, [0.1] * 7),
        {
            "request_seq": 5,
            "world_version": 4,
            "handoff_unix_ns": 2_000_000_000,
            "deadline_unix_ns": time.time_ns() + 1_000_000_000,
        },
    )
    assert result.valid
    assert result.diagnostics["trajectory_artifact_schema_version"] == 2
    assert result.diagnostics["collision_sphere_positions"].shape == (2, 56, 3)
    assert result.points[0].positions == [0.0] * 7


def test_space_time_backend_reads_candidate_specific_schema_v3_times(tmp_path):
    path = tmp_path / "trajectory-v3.npz"
    candidate_times = np.asarray([[0.0, 2.0, 6.0], [0.0, 4.0, 10.0]])
    np.savez(
        path,
        artifact_schema_version=np.asarray(3, dtype=np.int64),
        timing_schema_version=np.asarray(1, dtype=np.int64),
        best_trajectory_topk_index=np.asarray(1, dtype=np.int64),
        time_from_start=candidate_times[1],
        topk_time_from_start=candidate_times,
        joint_names=np.asarray(NAMES),
        collision_sphere_radii=np.full(56, 0.01, dtype=np.float32),
        topk_positions=np.zeros((2, 3, 7)),
        topk_velocities=np.zeros((2, 3, 7)),
        topk_accelerations=np.zeros((2, 3, 7)),
        topk_scores=np.asarray([0.2, 0.1]),
        topk_source_candidate_indices=np.asarray([3, 7]),
        topk_collision_sphere_positions=np.zeros((2, 3, 56, 3), dtype=np.float32),
    )
    backend = SpaceTimeMpdGlobalTrajectoryBackend("/tmp/not-used.sock")
    backend.client = SpaceTimeFakeClient(path)
    backend.warmup()
    snapshot = DynamicWorldSnapshot(
        4, "fr3_link0", 1_000_000_000, 20_000_000_000, ()
    )
    backend.update_world_snapshot(snapshot)
    result = backend.plan(
        StartState(NAMES, [0.0] * 7, [0.0] * 7, 1.0),
        JointTarget(NAMES, [0.1] * 7),
        {
            "request_seq": 5,
            "world_version": 4,
            "handoff_unix_ns": 2_000_000_000,
            "deadline_unix_ns": time.time_ns() + 1_000_000_000,
        },
    )

    assert result.valid
    assert result.diagnostics["candidate_specific_timing"] is True
    assert result.diagnostics["timing_schema_version"] == 1
    assert [point.time_from_start_s for point in result.points] == [0.0, 4.0, 10.0]
    first = result.diagnostics["top_k_candidates"][0]
    assert [point.time_from_start_s for point in first.points] == [0.0, 2.0, 6.0]
    assert first.diagnostics["duration_s"] == 6.0


def test_space_time_backend_rejects_phase4_worker_health():
    backend = SpaceTimeMpdGlobalTrajectoryBackend("/tmp/not-used.sock")
    backend.client = FakeClient("/tmp/not-used.npz")
    try:
        backend.warmup()
    except Exception as error:
        assert "Phase-5" in str(error)
    else:
        raise AssertionError("Phase-5 backend accepted a Phase-4 worker")
