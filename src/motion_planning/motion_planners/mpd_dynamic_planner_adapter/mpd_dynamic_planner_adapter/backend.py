"""ROS-side client/backend for the separate Phase-4 MPD worker."""

from __future__ import annotations

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
)
from mpd_planner_adapter.backend import (
    EXPECTED_JOINT_NAMES,
    MpdGlobalTrajectoryBackend,
)
from mpd_planner_adapter.client import MpdClientError, MpdWorkerClient

from .dynamic_world import DynamicWorldSnapshot


class DynamicMpdWorkerClient(MpdWorkerClient):
    def update_world(self, snapshot: DynamicWorldSnapshot) -> dict[str, Any]:
        return self.request(
            {
                "schema_version": 1,
                "op": "update_world",
                "world": snapshot.to_worker_dict(),
            }
        )

    def plan_dynamic(
        self,
        request: dict[str, Any],
        *,
        request_seq: int,
        world_version: int,
        trajectory_start_unix_ns: int,
        deadline_unix_ns: int | None,
    ) -> dict[str, Any]:
        return self.request(
            {
                "schema_version": 1,
                "op": "plan",
                "request_seq": request_seq,
                "world_version": world_version,
                "trajectory_start_unix_ns": trajectory_start_unix_ns,
                "deadline_unix_ns": deadline_unix_ns,
                "request": request,
            }
        )


class DynamicMpdGlobalTrajectoryBackend(MpdGlobalTrajectoryBackend):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.client = DynamicMpdWorkerClient(
            self.client.socket_path,
            timeout_s=self.client.timeout_s,
        )
        self.uploaded_world_version = 0

    def warmup(self) -> None:
        health = self.client.health()
        if health.get("status") != "OK" or health.get("state") != "READY":
            raise MpdClientError(f"dynamic MPD worker is not READY: {health.get('state')}")
        dense = health.get("engine", {}).get("dense_validation", {})
        if not (
            dense.get("fully_warmed")
            and dense.get("full_batch")
            and dense.get("pruning_used") is False
        ):
            raise MpdClientError("dynamic worker did not report full unpruned DenseCheck")

    def update_world_snapshot(self, snapshot: DynamicWorldSnapshot) -> None:
        response = self.client.update_world(snapshot)
        if response.get("status") != "OK" or response.get("world_version") != snapshot.version:
            raise MpdClientError(f"dynamic world update failed: {response}")
        self.uploaded_world_version = snapshot.version
        self.world_version = snapshot.version

    def plan(
        self,
        start_state: StartState,
        target: PoseTarget | JointTarget,
        options: dict[str, Any] | None = None,
    ) -> TrajectoryPlanResult:
        options = {} if options is None else dict(options)
        request_seq = int(options["request_seq"])
        world_version = int(options["world_version"])
        trajectory_start_ns = int(options["handoff_unix_ns"])
        deadline_ns = options.get("deadline_unix_ns")
        try:
            if world_version != self.uploaded_world_version:
                raise MpdClientError(
                    f"world {world_version} was not uploaded (loaded={self.uploaded_world_version})"
                )
            response = self.client.plan_dynamic(
                self._request(start_state, target, options),
                request_seq=request_seq,
                world_version=world_version,
                trajectory_start_unix_ns=trajectory_start_ns,
                deadline_unix_ns=deadline_ns,
            )
            if response.get("status") != "OK":
                return TrajectoryPlanResult(
                    valid=False,
                    reason=str(response.get("reason", response.get("status", "worker_error"))),
                    diagnostics={"worker_response": response},
                )
            if response.get("request_seq") != request_seq or response.get("world_version") != world_version:
                raise MpdClientError("dynamic response generation/version mismatch")
            if deadline_ns is not None and time.time_ns() >= int(deadline_ns):
                return TrajectoryPlanResult(valid=False, reason="deadline_expired_at_client")
            path = Path(str(response["trajectory_path"]))
            with np.load(path, allow_pickle=False) as data:
                positions = np.asarray(data["positions"], dtype=np.float64)
                velocities = np.asarray(data["velocities"], dtype=np.float64)
                stamps = np.asarray(data["time_from_start"], dtype=np.float64)
                names = tuple(str(item) for item in data["joint_names"].tolist())
                sphere_positions = np.asarray(
                    data["collision_sphere_positions"], dtype=np.float64
                )
                sphere_radii = np.asarray(data["collision_sphere_radii"], dtype=np.float64)
            if names != EXPECTED_JOINT_NAMES or positions.ndim != 2 or positions.shape[1] != 7:
                raise MpdClientError("dynamic trajectory joint contract mismatch")
            if velocities.shape != positions.shape or stamps.shape != (positions.shape[0],):
                raise MpdClientError("dynamic trajectory arrays are inconsistent")
            if sphere_positions.shape[:1] != positions.shape[:1] or sphere_positions.shape[-1] != 3:
                raise MpdClientError("collision sphere position array is inconsistent")
            if sphere_radii.shape != (sphere_positions.shape[1],):
                raise MpdClientError("collision sphere radii are inconsistent")
            arrays = (positions, velocities, stamps, sphere_positions, sphere_radii)
            if not all(np.isfinite(value).all() for value in arrays):
                raise MpdClientError("dynamic trajectory contains NaN or Inf")
            if len(stamps) < 2 or stamps[0] < 0.0 or np.any(np.diff(stamps) <= 0.0):
                raise MpdClientError("dynamic trajectory time is not strictly increasing")
            points = [
                TrajectoryPlanPoint(
                    positions=positions[index].tolist(),
                    velocities=velocities[index].tolist(),
                    time_from_start_s=float(stamps[index]),
                )
                for index in range(len(stamps))
            ]
            return TrajectoryPlanResult(
                valid=True,
                joint_names=list(EXPECTED_JOINT_NAMES),
                points=points,
                diagnostics={
                    "request_seq": request_seq,
                    "world_version": world_version,
                    "worker_elapsed_s": response.get("elapsed_sec"),
                    "engine_instance_id": response.get("engine_instance_id"),
                    "trajectory_path": str(path),
                    "handoff_unix_ns": trajectory_start_ns,
                    "collision_sphere_positions": sphere_positions,
                    "collision_sphere_radii": sphere_radii,
                },
            )
        except (KeyError, OSError, ValueError, MpdClientError) as error:
            return TrajectoryPlanResult(
                valid=False,
                reason=f"{type(error).__name__}: {error}",
                diagnostics={"request_seq": request_seq, "world_version": world_version},
            )
