"""ROS-free Phase-3 file/subprocess bridge for Marvin bimanual MPD."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
from typing import Mapping, Sequence

from .contract import JOINT_NAMES


REQUEST_SCHEMA = "marvin_bimanual_request/v2"
RESULT_SCHEMA = "marvin_bimanual_result/v2"


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical_json_sha256(value) -> str:
    encoded = json.dumps(
        value, allow_nan=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def isolated_conda_environment(
    source: Mapping[str, str] | None = None, *, conda_prefix: Path
) -> dict[str, str]:
    """Remove ROS Python injection while retaining native library lookup."""
    child = dict(os.environ if source is None else source)
    child.pop("PYTHONPATH", None)
    child.pop("PYTHONHOME", None)
    conda_lib = str(Path(conda_prefix).expanduser() / "lib")
    inherited = child.get("LD_LIBRARY_PATH")
    child["LD_LIBRARY_PATH"] = f"{conda_lib}:{inherited}" if inherited else conda_lib
    return child


@dataclass(frozen=True)
class JointStateSnapshot:
    positions: tuple[float, ...]
    received_monotonic_ns: int
    source_stamp_ns: int

    def require_fresh(self, *, now_monotonic_ns: int, max_age_s: float) -> None:
        age_ns = int(now_monotonic_ns) - self.received_monotonic_ns
        if age_ns < 0 or age_ns > int(float(max_age_s) * 1e9):
            raise ValueError(f"joint state is stale ({age_ns / 1e9:.3f}s)")


def canonical_joint_state(
    names: Sequence[str],
    positions: Sequence[float],
    *,
    received_monotonic_ns: int,
    source_stamp_ns: int = 0,
) -> JointStateSnapshot:
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError("joint state names/positions must be unique and same length")
    mapping = dict(zip(names, positions))
    missing = [name for name in JOINT_NAMES if name not in mapping]
    if missing:
        raise ValueError(f"joint state is missing canonical joints: {missing}")
    ordered = tuple(float(mapping[name]) for name in JOINT_NAMES)
    if not all(math.isfinite(value) for value in ordered):
        raise ValueError("joint state contains NaN or Inf")
    return JointStateSnapshot(ordered, int(received_monotonic_ns), int(source_stamp_ns))


def build_request(
    *,
    request_id: str,
    q_start: Sequence[float],
    q_goal: Sequence[float] | None,
    left_goal_pose: Mapping | None,
    right_goal_pose: Mapping | None,
    deadline_unix_ns: int,
    seed: int,
    tf_stamp_ns: int,
) -> dict:
    request = {
        "schema": REQUEST_SCHEMA,
        "request_id": str(request_id),
        "task_mode": "dual_independent",
        "runtime_mode": "snapshot_no_time",
        "robot_model": "marvin_bimanual",
        "planning_frame": "world",
        "scene_id": "EnvWarehouseMarvinBimanual",
        "scene_version": "marvin_warehouse_v2",
        "joint_names": list(JOINT_NAMES),
        "q_start": [float(value) for value in q_start],
        "q_goal": None if q_goal is None else [float(value) for value in q_goal],
        "q_velocity_start": [0.0] * 14,
        "q_acceleration_start": [0.0] * 14,
        "left_goal_pose": left_goal_pose,
        "right_goal_pose": right_goal_pose,
        "world_version": 0,
        "deadline_unix_ns": int(deadline_unix_ns),
        "seed": int(seed),
        "scene": {"tf_stamp_ns": int(tf_stamp_ns), "source": "ros2_one_shot"},
    }
    if q_goal is None and (left_goal_pose is None or right_goal_pose is None):
        raise ValueError("dual independent request needs q_goal or both EE poses")
    return request


def _finite_matrix(value, *, columns: int, field: str) -> list[list[float]]:
    if not isinstance(value, list) or len(value) < 2:
        raise ValueError(f"{field} must contain at least two rows")
    rows = []
    for index, row in enumerate(value):
        if not isinstance(row, list) or len(row) != columns:
            raise ValueError(f"{field}[{index}] must contain {columns} values")
        converted = [float(item) for item in row]
        if not all(math.isfinite(item) for item in converted):
            raise ValueError(f"{field}[{index}] contains NaN or Inf")
        rows.append(converted)
    return rows


def validate_artifact(
    output_dir: Path,
    request: Mapping,
    *,
    max_start_error_rad: float = 1e-5,
) -> dict:
    """Reload and bind every Phase-3 file before ROS may expose a trajectory."""
    import numpy as np

    output_dir = Path(output_dir)
    result_path = output_dir / "result.json"
    trajectory_path = output_dir / "trajectory.npz"
    scene_path = output_dir / "scene.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    if result.get("schema") != RESULT_SCHEMA or result.get("status") != "success":
        raise ValueError(f"MPD result is not successful: {result.get('status')}")
    if result.get("request_id") != request.get("request_id"):
        raise ValueError("result request_id does not match request")
    if tuple(result.get("joint_names", ())) != JOINT_NAMES:
        raise ValueError("result joint order is not canonical Marvin order")
    artifacts = result.get("artifacts") or {}
    for key, path in (
        ("trajectory_sha256", trajectory_path),
        ("scene_file_sha256", scene_path),
    ):
        if artifacts.get(key) != sha256_file(path):
            raise ValueError(f"{key} does not match artifact bytes")
    request_path = output_dir / "request.json"
    if artifacts.get("request_sha256") != sha256_file(request_path):
        raise ValueError("request_sha256 does not match request.json")

    scene = json.loads(scene_path.read_text(encoding="utf-8"))
    result_scene = result.get("scene") or {}
    if result_scene.get("scene_sha256") and result_scene["scene_sha256"] != canonical_json_sha256(scene):
        raise ValueError("result scene hash does not match scene payload")
    if request.get("scene_hash") and request["scene_hash"] != result_scene.get("scene_sha256"):
        raise ValueError("result scene hash does not match requested scene hash")
    result_model = result.get("model") or {}
    if request.get("checkpoint_hash") and request["checkpoint_hash"] != result_model.get("checkpoint_sha256"):
        raise ValueError("result checkpoint hash does not match request")

    positions = _finite_matrix(result.get("positions"), columns=14, field="positions")
    velocities = _finite_matrix(result.get("velocities"), columns=14, field="velocities")
    accelerations = _finite_matrix(result.get("accelerations"), columns=14, field="accelerations")
    if len(positions) != len(velocities) or len(positions) != len(accelerations):
        raise ValueError("trajectory derivative arrays do not match positions")
    times = [float(item) for item in result.get("time_from_start", ())]
    if len(times) != len(positions) or not all(math.isfinite(item) for item in times):
        raise ValueError("time_from_start shape/values are invalid")
    if abs(times[0]) > 1e-9 or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("time_from_start must start at zero and strictly increase")
    start_error = max(abs(a - b) for a, b in zip(positions[0], request["q_start"]))
    if start_error > float(max_start_error_rad):
        raise ValueError(f"trajectory start error {start_error:.6g} exceeds limit")

    with np.load(trajectory_path, allow_pickle=False) as archive:
        if archive["joint_names"].tolist() != list(JOINT_NAMES):
            raise ValueError("trajectory.npz joint order is invalid")
        for key, expected in (
            ("positions", positions),
            ("velocities", velocities),
            ("accelerations", accelerations),
            ("time_from_start", times),
        ):
            actual = np.asarray(archive[key], dtype=np.float64)
            wanted = np.asarray(expected, dtype=np.float64)
            if actual.shape != wanted.shape or not np.array_equal(actual, wanted):
                raise ValueError(f"trajectory.npz {key} differs from result.json")
    return result


class OneShotMpdPlanner:
    def __init__(
        self,
        *,
        conda: Path,
        conda_env: str,
        conda_prefix: Path,
        infer_once: Path,
        output_root: Path,
        device: str,
        timeout_s: float,
    ) -> None:
        self.conda = Path(conda)
        self.conda_env = str(conda_env)
        self.conda_prefix = Path(conda_prefix)
        self.infer_once = Path(infer_once)
        self.output_root = Path(output_root).expanduser()
        self.device = str(device)
        self.timeout_s = float(timeout_s)

    def plan(self, request: Mapping) -> tuple[dict, Path]:
        request_id = str(request["request_id"])
        output_dir = self.output_root / request_id
        output_dir.mkdir(parents=True, exist_ok=False)
        request_path = output_dir / "request.json"
        request_path.write_text(
            json.dumps(request, allow_nan=False, indent=2) + "\n", encoding="utf-8"
        )
        completed = subprocess.run(
            [
                str(self.conda),
                "run",
                "--no-capture-output",
                "-n",
                self.conda_env,
                "python",
                str(self.infer_once),
                "--request",
                str(request_path),
                "--output-dir",
                str(output_dir),
                "--device",
                self.device,
            ],
            capture_output=True,
            text=True,
            timeout=self.timeout_s,
            check=False,
            env=isolated_conda_environment(conda_prefix=self.conda_prefix),
        )
        if completed.returncode != 0:
            message = completed.stderr.strip() or completed.stdout.strip()
            raise RuntimeError(message or f"MPD one-shot exited {completed.returncode}")
        return validate_artifact(output_dir, request), output_dir
