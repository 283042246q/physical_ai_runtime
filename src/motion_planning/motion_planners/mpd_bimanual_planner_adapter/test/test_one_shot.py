import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from mpd_bimanual_planner_adapter.contract import JOINT_NAMES
from mpd_bimanual_planner_adapter.one_shot import (
    canonical_joint_state,
    isolated_conda_environment,
    validate_artifact,
)


def test_joint_state_is_reordered_checked_and_aged():
    names = list(reversed(JOINT_NAMES))
    snapshot = canonical_joint_state(
        names, list(range(14)), received_monotonic_ns=1_000_000_000
    )
    assert snapshot.positions == tuple(reversed(range(14)))
    snapshot.require_fresh(now_monotonic_ns=1_100_000_000, max_age_s=0.2)
    with pytest.raises(ValueError, match="stale"):
        snapshot.require_fresh(now_monotonic_ns=2_000_000_000, max_age_s=0.2)
    with pytest.raises(ValueError, match="missing"):
        canonical_joint_state(names[:-1], list(range(13)), received_monotonic_ns=0)


def test_conda_environment_removes_ros_python_injection():
    child = isolated_conda_environment(
        {"PYTHONPATH": "/ros", "PYTHONHOME": "/ros/home", "LD_LIBRARY_PATH": "/native"},
        conda_prefix=Path("/conda/env"),
    )
    assert "PYTHONPATH" not in child and "PYTHONHOME" not in child
    assert child["LD_LIBRARY_PATH"] == "/conda/env/lib:/native"


def test_artifact_validation_binds_all_files(tmp_path):
    request = {"request_id": "phase3", "q_start": [0.0] * 14}
    request_path = tmp_path / "request.json"
    request_path.write_text(json.dumps(request) + "\n")
    scene = {"frame_id": "world", "obstacles": []}
    scene_path = tmp_path / "scene.json"
    scene_path.write_text(json.dumps(scene, indent=2) + "\n")
    positions = np.asarray([[0.0] * 14, [0.1] * 14])
    velocities = np.zeros_like(positions)
    accelerations = np.zeros_like(positions)
    times = np.asarray([0.0, 1.0])
    trajectory_path = tmp_path / "trajectory.npz"
    np.savez_compressed(
        trajectory_path,
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
        time_from_start=times,
        joint_names=np.asarray(JOINT_NAMES),
    )
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    canonical_scene = hashlib.sha256(
        json.dumps(scene, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    result = {
        "schema": "marvin_bimanual_result/v2",
        "status": "success",
        "request_id": "phase3",
        "joint_names": list(JOINT_NAMES),
        "positions": positions.tolist(),
        "velocities": velocities.tolist(),
        "accelerations": accelerations.tolist(),
        "time_from_start": times.tolist(),
        "scene": {"scene_sha256": canonical_scene},
        "artifacts": {
            "request_sha256": digest(request_path),
            "trajectory_sha256": digest(trajectory_path),
            "scene_file_sha256": digest(scene_path),
        },
    }
    (tmp_path / "result.json").write_text(json.dumps(result))
    assert validate_artifact(tmp_path, request)["request_id"] == "phase3"
    with trajectory_path.open("ab") as stream:
        stream.write(b"corrupt")
    with pytest.raises(ValueError, match="trajectory_sha256"):
        validate_artifact(tmp_path, request)
