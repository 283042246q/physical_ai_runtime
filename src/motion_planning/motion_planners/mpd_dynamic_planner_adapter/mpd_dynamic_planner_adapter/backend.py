"""ROS-side client/backend for the separate Phase-4 and Phase-5 MPD workers."""

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
                error = response.get("error")
                detail = (
                    f"{error.get('type')}: {error.get('message')}"
                    if isinstance(error, dict)
                    else str(response.get("reason", response.get("status", "worker_error")))
                )
                return TrajectoryPlanResult(
                    valid=False,
                    reason=detail,
                    diagnostics={"worker_response": response},
                )
            if response.get("request_seq") != request_seq or response.get("world_version") != world_version:
                raise MpdClientError("dynamic response generation/version mismatch")
            if deadline_ns is not None and time.time_ns() >= int(deadline_ns):
                return TrajectoryPlanResult(valid=False, reason="deadline_expired_at_client")
            path = Path(str(response["trajectory_path"]))
            with np.load(path, allow_pickle=False) as data:
                artifact_response = response.get("trajectory_artifact", {})
                schema_version = int(
                    np.asarray(data["artifact_schema_version"]).item()
                    if "artifact_schema_version" in data
                    else artifact_response.get("schema_version", 1)
                )
                if schema_version not in (1, 2, 3):
                    raise MpdClientError(
                        f"unsupported dynamic trajectory schema v{schema_version}"
                    )
                stamps = np.asarray(data["time_from_start"], dtype=np.float64)
                names = tuple(str(item) for item in data["joint_names"].tolist())
                sphere_radii = np.asarray(data["collision_sphere_radii"], dtype=np.float64)
                topk_positions = np.asarray(data["topk_positions"], dtype=np.float64)
                topk_velocities = np.asarray(data["topk_velocities"], dtype=np.float64)
                topk_accelerations = np.asarray(data["topk_accelerations"], dtype=np.float64)
                topk_scores = np.asarray(data["topk_scores"], dtype=np.float64)
                topk_source_indices = np.asarray(
                    data["topk_source_candidate_indices"], dtype=np.int64
                )
                topk_sphere_positions = np.asarray(
                    data["topk_collision_sphere_positions"], dtype=np.float64
                )
                best_index = int(
                    np.asarray(data["best_trajectory_topk_index"]).item()
                    if "best_trajectory_topk_index" in data
                    else artifact_response.get("best_trajectory_topk_index", 0)
                )
                if best_index < 0 or best_index >= topk_positions.shape[0]:
                    raise MpdClientError("best trajectory top-K index is out of range")
                timing_schema_version = 0
                if schema_version == 3:
                    timing_schema_version = int(
                        np.asarray(data["timing_schema_version"]).item()
                    )
                    if timing_schema_version != 1:
                        raise MpdClientError(
                            f"unsupported timing schema v{timing_schema_version}"
                        )
                    topk_stamps = np.asarray(
                        data["topk_time_from_start"], dtype=np.float64
                    )
                else:
                    topk_stamps = np.broadcast_to(
                        stamps, (topk_positions.shape[0], stamps.shape[0])
                    ).copy()
                positions = np.asarray(
                    data["positions"] if "positions" in data else topk_positions[best_index],
                    dtype=np.float64,
                )
                velocities = np.asarray(
                    data["velocities"] if "velocities" in data else topk_velocities[best_index],
                    dtype=np.float64,
                )
                accelerations = np.asarray(
                    data["accelerations"]
                    if "accelerations" in data
                    else topk_accelerations[best_index],
                    dtype=np.float64,
                )
                sphere_positions = np.asarray(
                    data["collision_sphere_positions"]
                    if "collision_sphere_positions" in data
                    else topk_sphere_positions[best_index],
                    dtype=np.float64,
                )
            if names != EXPECTED_JOINT_NAMES or positions.ndim != 2 or positions.shape[1] != 7:
                raise MpdClientError("dynamic trajectory joint contract mismatch")
            if (
                velocities.shape != positions.shape
                or accelerations.shape != positions.shape
                or stamps.shape != (positions.shape[0],)
            ):
                raise MpdClientError("dynamic trajectory arrays are inconsistent")
            if sphere_positions.shape[:1] != positions.shape[:1] or sphere_positions.shape[-1] != 3:
                raise MpdClientError("collision sphere position array is inconsistent")
            if sphere_radii.shape != (sphere_positions.shape[1],):
                raise MpdClientError("collision sphere radii are inconsistent")
            topk_shape = topk_positions.shape
            if (
                len(topk_shape) != 3
                or topk_shape[1:] != positions.shape
                or topk_stamps.shape != topk_shape[:2]
                or topk_velocities.shape != topk_shape
                or topk_accelerations.shape != topk_shape
                or topk_scores.shape != (topk_shape[0],)
                or topk_source_indices.shape != (topk_shape[0],)
                or topk_sphere_positions.shape
                != (topk_shape[0], *sphere_positions.shape)
            ):
                raise MpdClientError("dynamic top-K trajectory arrays are inconsistent")
            arrays = (
                positions,
                velocities,
                accelerations,
                stamps,
                sphere_positions,
                sphere_radii,
                topk_positions,
                topk_velocities,
                topk_accelerations,
                topk_stamps,
                topk_scores,
                topk_sphere_positions,
            )
            if not all(np.isfinite(value).all() for value in arrays):
                raise MpdClientError("dynamic trajectory contains NaN or Inf")
            if len(stamps) < 2 or stamps[0] < 0.0 or np.any(np.diff(stamps) <= 0.0):
                raise MpdClientError("dynamic trajectory time is not strictly increasing")
            if (
                np.any(topk_stamps[:, 0] < 0.0)
                or np.any(np.diff(topk_stamps, axis=1) <= 0.0)
            ):
                raise MpdClientError(
                    "dynamic top-K trajectory time is not strictly increasing"
                )
            if schema_version == 3 and not np.allclose(
                stamps, topk_stamps[best_index], rtol=0.0, atol=1e-9
            ):
                raise MpdClientError(
                    "selected trajectory time does not match its top-K timing"
                )
            q_start = np.asarray(self._request(start_state, target, options)["q_pos_start"])
            dq_start = np.asarray(self._request(start_state, target, options)["q_vel_start"])
            ddq_start = np.asarray(options.get("q_acc_start", np.zeros(7)), dtype=np.float64)
            boundary_errors = np.asarray(
                [
                    np.max(np.abs(topk_positions[:, 0] - q_start)),
                    np.max(np.abs(topk_velocities[:, 0] - dq_start)),
                    np.max(np.abs(topk_accelerations[:, 0] - ddq_start)),
                ]
            )
            # Phase-5 derivatives are evaluated by the CUDA float32 timing
            # chain rule before export; allow its one-ULP-scale endpoint
            # acceleration residue without weakening the Phase-4 contract.
            boundary_tolerance = 2e-5 if schema_version == 3 else 1e-5
            if np.any(boundary_errors > boundary_tolerance):
                raise MpdClientError(
                    "dynamic top-K q/dq/ddq start boundary mismatch: "
                    f"{boundary_errors.tolist()}"
                )

            candidates = []
            for candidate_index in range(topk_shape[0]):
                candidate_stamps = topk_stamps[candidate_index]
                points = [
                    TrajectoryPlanPoint(
                        positions=topk_positions[candidate_index, index].tolist(),
                        velocities=topk_velocities[candidate_index, index].tolist(),
                        accelerations=topk_accelerations[candidate_index, index].tolist(),
                        time_from_start_s=float(candidate_stamps[index]),
                    )
                    for index in range(len(candidate_stamps))
                ]
                candidates.append(
                    TrajectoryPlanResult(
                        valid=True,
                        joint_names=list(EXPECTED_JOINT_NAMES),
                        points=points,
                        diagnostics={
                            "mpd_selection_score": float(topk_scores[candidate_index]),
                            "mpd_source_candidate_index": int(
                                topk_source_indices[candidate_index]
                            ),
                            "collision_sphere_positions": topk_sphere_positions[
                                candidate_index
                            ],
                            "collision_sphere_radii": sphere_radii,
                            "trajectory_path": str(path),
                            "duration_s": float(candidate_stamps[-1]),
                            "timing_schema_version": timing_schema_version,
                        },
                    )
                )
            primary = candidates[best_index]
            primary.diagnostics.update(
                {
                    "request_seq": request_seq,
                    "world_version": world_version,
                    "worker_elapsed_s": response.get("elapsed_sec"),
                    "engine_instance_id": response.get("engine_instance_id"),
                    "trajectory_path": str(path),
                    "handoff_unix_ns": trajectory_start_ns,
                    "collision_sphere_positions": sphere_positions,
                    "collision_sphere_radii": sphere_radii,
                    "top_k_candidates": candidates,
                    "top_k_count": len(candidates),
                    "start_boundary_errors": boundary_errors,
                    "start_boundary_tolerance": boundary_tolerance,
                    "trajectory_artifact_schema_version": schema_version,
                    "timing_schema_version": timing_schema_version,
                    "candidate_specific_timing": schema_version == 3,
                    "best_trajectory_topk_index": best_index,
                },
            )
            return primary
        except (KeyError, OSError, ValueError, MpdClientError) as error:
            return TrajectoryPlanResult(
                valid=False,
                reason=f"{type(error).__name__}: {error}",
                diagnostics={"request_seq": request_seq, "world_version": world_version},
            )


class SpaceTimeMpdGlobalTrajectoryBackend(DynamicMpdGlobalTrajectoryBackend):
    """Phase-5 backend that rejects workers without the space-time contract."""

    def __init__(self, *args, expected_timing_mode: str = "phase5_joint", **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.expected_timing_mode = str(expected_timing_mode)

    def warmup(self) -> None:
        super().warmup()
        health = self.client.health()
        space_time = health.get("engine", {}).get("space_time", {})
        if not (
            space_time.get("enabled")
            and space_time.get("candidate_specific_time")
            and space_time.get("trajectory_schema_version") == 3
            and space_time.get("timing_schema_version") == 1
        ):
            raise MpdClientError(
                "worker did not report the Phase-5 candidate-specific timing contract"
            )
        actual_mode = str(space_time.get("mode", ""))
        if self.expected_timing_mode and actual_mode != self.expected_timing_mode:
            raise MpdClientError(
                f"worker timing mode {actual_mode!r} does not match "
                f"{self.expected_timing_mode!r}"
            )
