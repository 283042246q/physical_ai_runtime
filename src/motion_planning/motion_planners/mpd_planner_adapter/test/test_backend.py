import time

import numpy as np

from manipulation_motion_planning.contracts import (
    JointTarget,
    StartState,
    World,
    WorldBox,
    WorldPose,
)
from mpd_planner_adapter.backend import EXPECTED_JOINT_NAMES, MpdGlobalTrajectoryBackend


class FakeClient:
    def __init__(self, trajectory_path):
        self.trajectory_path = trajectory_path
        self.last = None

    def health(self):
        return {
            "schema_version": 1,
            "status": "OK",
            "state": "READY",
            "engine": {"dense_validation": {"fully_warmed": True}},
        }

    def plan(self, request, **options):
        self.last = (request, options)
        return {
            "schema_version": 1,
            "status": "OK",
            "request_seq": options["request_seq"],
            "world_version": options["world_version"],
            "trajectory_path": str(self.trajectory_path),
            "engine_instance_id": "fixed-engine",
            "elapsed_sec": 0.2,
        }


def test_backend_translates_and_validates_worker_result(tmp_path):
    trajectory = tmp_path / "trajectory.npz"
    np.savez(
        trajectory,
        positions=np.vstack([np.zeros(7), np.ones(7)]),
        velocities=np.zeros((2, 7)),
        time_from_start=np.asarray([0.0, 1.0]),
        joint_names=np.asarray(EXPECTED_JOINT_NAMES),
    )
    backend = MpdGlobalTrajectoryBackend("/unused")
    fake = FakeClient(trajectory)
    backend.client = fake
    backend.warmup()
    state = StartState(list(EXPECTED_JOINT_NAMES), [0.0] * 7, [0.1] * 7, 1.0)
    target = JointTarget(list(EXPECTED_JOINT_NAMES), [0.2] * 7)
    result = backend.plan(
        state,
        target,
        {
            "request_seq": 7,
            "world_version": 3,
            "deadline_unix_ns": time.time_ns() + 1_000_000_000,
        },
    )
    assert result.valid
    assert len(result.points) == 2
    assert fake.last[0]["q_vel_start"] == [0.1] * 7
    assert fake.last[0]["q_pos_goal"] == [0.2] * 7
    assert result.diagnostics["engine_instance_id"] == "fixed-engine"


def test_phase2_rejects_dynamic_geometry():
    backend = MpdGlobalTrajectoryBackend("/unused")
    world = World(
        boxes=[WorldBox("box", WorldPose((0.0, 0.0, 0.0)), (1.0, 1.0, 1.0))]
    )
    try:
        backend.update_world(world)
    except ValueError as error:
        assert "static scene" in str(error)
    else:
        raise AssertionError("dynamic world must be rejected in Phase 2")
