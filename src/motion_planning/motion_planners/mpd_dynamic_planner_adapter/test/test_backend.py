import time

import numpy as np

from manipulation_motion_planning.contracts import JointTarget, StartState

from mpd_dynamic_planner_adapter.backend import DynamicMpdGlobalTrajectoryBackend
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


def test_dynamic_backend_uploads_exact_version_and_reads_collision_spheres(tmp_path):
    path = tmp_path / "trajectory.npz"
    np.savez(
        path,
        positions=np.zeros((2, 7)),
        velocities=np.zeros((2, 7)),
        time_from_start=np.asarray([0.0, 10.0]),
        joint_names=np.asarray(NAMES),
        collision_sphere_positions=np.zeros((2, 56, 3)),
        collision_sphere_radii=np.full(56, 0.01),
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
