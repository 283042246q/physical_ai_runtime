"""GlobalTrajectoryBackend implementation backed by the resident MPD worker."""

from __future__ import annotations

import math
from pathlib import Path
import time
from typing import Any

import numpy as np

from manipulation_motion_planning.contracts import (
    JointTarget,
    PoseTarget,
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
    World,
)

from .client import MpdClientError, MpdWorkerClient


EXPECTED_JOINT_NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))


def _ordered(values: list[float], names: list[str]) -> list[float]:
    if len(names) != len(values) or set(names) != set(EXPECTED_JOINT_NAMES):
        raise ValueError("joint state must contain each ordered FR3 arm joint exactly once")
    by_name = dict(zip(names, values))
    ordered = [float(by_name[name]) for name in EXPECTED_JOINT_NAMES]
    if not all(math.isfinite(value) for value in ordered):
        raise ValueError("joint state contains NaN or Inf")
    return ordered


def _world_has_geometry(world: World) -> bool:
    return any(
        (
            world.boxes,
            world.spheres,
            world.capsules,
            world.meshes,
            world.voxel_grids,
            world.point_clouds,
        )
    )


class MpdGlobalTrajectoryBackend:
    """Synchronous backend; `LatestOnlyPlanner` keeps it off ROS callbacks."""

    def __init__(
        self,
        socket_path: str | Path,
        *,
        scene_id: str = "EnvWarehouseExtraObjectsV00",
        seed: int = 0,
        timeout_s: float = 2.0,
    ) -> None:
        self.client = MpdWorkerClient(socket_path, timeout_s=timeout_s)
        self.scene_id = scene_id
        self.seed = int(seed)
        self.world_version = 0

    def warmup(self) -> None:
        health = self.client.health()
        if health.get("status") != "OK" or health.get("state") != "READY":
            raise MpdClientError(f"MPD worker is not READY: {health.get('state')}")
        engine = health.get("engine", {})
        dense_validation = engine.get("dense_validation", {})
        if not dense_validation.get("fully_warmed", False):
            raise MpdClientError("MPD worker reported READY without full warmup")

    def update_world(self, world: World) -> None:
        if _world_has_geometry(world):
            raise ValueError("Phase 2 MPD adapter only supports its configured static scene")
        self.world_version += 1

    def _request(
        self, start_state: StartState, target: PoseTarget | JointTarget, options: dict[str, Any]
    ) -> dict[str, Any]:
        q_start = _ordered(start_state.positions, start_state.joint_names)
        if start_state.velocities is None:
            dq_start = [0.0] * 7
        else:
            dq_start = _ordered(start_state.velocities, start_state.joint_names)
        request: dict[str, Any] = {
            "schema_version": 1,
            "request_id": str(options.get("request_id", "ros-replan")),
            "robot_model": "franka_fr3",
            "planning_frame": "fr3_link0",
            "joint_names": list(EXPECTED_JOINT_NAMES),
            "goal_type": "joint" if isinstance(target, JointTarget) else "cartesian",
            "q_pos_start": q_start,
            "q_vel_start": dq_start,
            "q_acc_start": list(options.get("q_acc_start", [0.0] * 7)),
            "q_vel_goal": list(options.get("q_vel_goal", [0.0] * 7)),
            "q_acc_goal": list(options.get("q_acc_goal", [0.0] * 7)),
            "joint_state_stamp": start_state.stamp_s,
            "scene_id": self.scene_id,
            "seed": int(options.get("seed", self.seed)),
        }
        if isinstance(target, JointTarget):
            request["q_pos_goal"] = _ordered(target.positions, target.joint_names)
        else:
            x, y, z = target.position_xyz
            qw, qx, qy, qz = target.orientation_wxyz
            request["ee_pose_goal"] = [x, y, z, qx, qy, qz, qw]
        return request

    def plan(
        self,
        start_state: StartState,
        target: PoseTarget | JointTarget,
        options: dict[str, Any] | None = None,
    ) -> TrajectoryPlanResult:
        options = {} if options is None else dict(options)
        request_seq = int(options["request_seq"])
        requested_world = int(options.get("world_version", self.world_version))
        deadline_unix_ns = options.get("deadline_unix_ns")
        try:
            response = self.client.plan(
                self._request(start_state, target, options),
                request_seq=request_seq,
                world_version=requested_world,
                deadline_unix_ns=deadline_unix_ns,
            )
            if response.get("status") != "OK":
                return TrajectoryPlanResult(
                    valid=False,
                    reason=str(response.get("reason", response.get("status", "worker_error"))),
                    diagnostics={"worker_response": response},
                )
            if response.get("request_seq") != request_seq:
                raise MpdClientError("response request_seq mismatch")
            if response.get("world_version") != requested_world:
                raise MpdClientError("response world_version mismatch")
            if deadline_unix_ns is not None and time.time_ns() >= int(deadline_unix_ns):
                return TrajectoryPlanResult(valid=False, reason="deadline_expired_at_client")
            trajectory_path = Path(str(response["trajectory_path"]))
            with np.load(trajectory_path, allow_pickle=False) as data:
                positions = np.asarray(data["positions"], dtype=np.float64)
                velocities = np.asarray(data["velocities"], dtype=np.float64)
                stamps = np.asarray(data["time_from_start"], dtype=np.float64)
                names = tuple(str(item) for item in data["joint_names"].tolist())
            if names != EXPECTED_JOINT_NAMES:
                raise MpdClientError("trajectory joint names mismatch")
            if positions.ndim != 2 or positions.shape[1] != 7:
                raise MpdClientError(f"invalid trajectory shape: {positions.shape}")
            if velocities.shape != positions.shape or stamps.shape != (positions.shape[0],):
                raise MpdClientError("trajectory arrays have inconsistent shapes")
            if not (
                np.isfinite(positions).all()
                and np.isfinite(velocities).all()
                and np.isfinite(stamps).all()
            ):
                raise MpdClientError("trajectory contains NaN or Inf")
            if positions.shape[0] < 2 or stamps[0] < 0.0 or np.any(np.diff(stamps) <= 0.0):
                raise MpdClientError("trajectory time is not strictly increasing")
            points = [
                TrajectoryPlanPoint(
                    positions=positions[i].tolist(),
                    velocities=velocities[i].tolist(),
                    time_from_start_s=float(stamps[i]),
                )
                for i in range(positions.shape[0])
            ]
            return TrajectoryPlanResult(
                valid=True,
                joint_names=list(EXPECTED_JOINT_NAMES),
                points=points,
                diagnostics={
                    "request_seq": request_seq,
                    "world_version": requested_world,
                    "worker_elapsed_s": response.get("elapsed_sec"),
                    "engine_instance_id": response.get("engine_instance_id"),
                    "trajectory_path": str(trajectory_path),
                    "handoff_unix_ns": options.get("handoff_unix_ns"),
                },
            )
        except (KeyError, OSError, ValueError) as error:
            return TrajectoryPlanResult(
                valid=False,
                reason=f"{type(error).__name__}: {error}",
                diagnostics={"request_seq": request_seq, "world_version": requested_world},
            )
