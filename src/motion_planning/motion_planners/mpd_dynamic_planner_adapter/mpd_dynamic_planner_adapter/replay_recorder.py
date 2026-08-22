"""Passive Phase-4 trace recorder for deterministic IsaacLab replay."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import threading
import time
from typing import Any

import numpy as np

from manipulation_motion_planning.contracts import StartState, TrajectoryPlanResult

from .dynamic_world import DynamicWorldSnapshot


def _best_positions_from_archive(data) -> np.ndarray:
    if "positions" in data:
        return np.asarray(data["positions"], dtype=np.float64)
    schema_version = int(np.asarray(data["artifact_schema_version"]).item())
    if schema_version != 2:
        raise ValueError("omitted positions require trajectory schema v2")
    best_index = int(np.asarray(data["best_trajectory_topk_index"]).item())
    topk_positions = np.asarray(data["topk_positions"], dtype=np.float64)
    if best_index < 0 or best_index >= len(topk_positions):
        raise ValueError("best trajectory top-K index is out of range")
    return topk_positions[best_index]


@dataclass
class _RecordedPlan:
    plan_id: str
    trajectory: str
    created_s: float
    start_s: float
    status: str
    active_from_s: float | None = None
    active_until_s: float | None = None
    handoff_s: float | None = None
    duration_s: float = 0.0
    phase_timing: dict[str, float] | None = None


class DynamicReplayRecorder:
    """Record planner facts without participating in planning or execution."""

    def __init__(
        self,
        output_dir: str | Path,
        *,
        env_name: str,
        static_scene_path: str | Path,
        frame_id: str = "fr3_link0",
    ) -> None:
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.env_name = str(env_name)
        self.frame_id = str(frame_id)
        scene_path = Path(static_scene_path).expanduser().resolve()
        scene = json.loads(scene_path.read_text(encoding="utf-8"))
        if not isinstance(scene, dict) or not isinstance(scene.get("obstacles"), list):
            raise ValueError("static replay scene must contain an obstacles list")
        if scene.get("env_name") not in (None, self.env_name):
            raise ValueError("static replay scene env_name does not match recorder env_name")
        self._static_scene = scene
        self._lock = threading.Lock()
        self._episode_start_ns: int | None = None
        self._latest_state: StartState | None = None
        self._initial_q: list[float] | None = None
        self._worlds: list[dict[str, Any]] = []
        self._plans: dict[str, _RecordedPlan] = {}
        self._generation_to_plan: dict[int, str] = {}
        self._active_plan_id: str | None = None
        self._events: list[dict[str, Any]] = []
        self._next_plan_index = 0
        self._closed = False

    @property
    def manifest_path(self) -> Path:
        return self.output_dir / "replay-manifest.json"

    def _relative_s(self, unix_ns: int) -> float:
        if self._episode_start_ns is None:
            raise RuntimeError("the first dynamic world snapshot has not been recorded")
        return max(0.0, (int(unix_ns) - self._episode_start_ns) * 1e-9)

    def record_state(self, state: StartState) -> None:
        with self._lock:
            self._latest_state = state
            if self._episode_start_ns is not None and self._initial_q is None:
                self._initial_q = list(state.positions)

    def record_world(self, snapshot: DynamicWorldSnapshot) -> None:
        with self._lock:
            if self._episode_start_ns is None:
                self._episode_start_ns = snapshot.stamp_unix_ns
                if self._latest_state is not None:
                    self._initial_q = list(self._latest_state.positions)
            time_s = self._relative_s(snapshot.stamp_unix_ns)
            if self._worlds and time_s <= self._worlds[-1]["time_s"]:
                return
            worker_world = snapshot.to_worker_dict()
            self._worlds.append(
                {
                    "time_s": time_s,
                    "world_version": snapshot.version,
                    "valid_until_s": self._relative_s(snapshot.valid_until_unix_ns),
                    "objects": worker_world["objects"],
                }
            )

    def _new_plan_id(self) -> str:
        plan_id = f"plan-{self._next_plan_index:04d}"
        self._next_plan_index += 1
        return plan_id

    @staticmethod
    def _arrays(
        result: TrajectoryPlanResult,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        positions = np.asarray([point.positions for point in result.points], dtype=np.float64)
        times = np.asarray([point.time_from_start_s for point in result.points], dtype=np.float64)
        velocities = np.asarray(
            [point.velocities if point.velocities is not None else np.zeros(positions.shape[1]) for point in result.points],
            dtype=np.float64,
        )
        accelerations = np.asarray(
            [
                point.accelerations
                if point.accelerations is not None
                else np.zeros(positions.shape[1])
                for point in result.points
            ],
            dtype=np.float64,
        )
        if positions.ndim != 2 or positions.shape[0] < 2 or positions.shape[1] not in (7, 9):
            raise ValueError("recorded trajectory positions must have shape [H,7] or [H,9]")
        if times.shape != (len(positions),) or np.any(np.diff(times) <= 0.0):
            raise ValueError("recorded trajectory time must be strictly increasing")
        return positions, velocities, accelerations, times

    def _write_plan(self, plan_id: str, result: TrajectoryPlanResult) -> tuple[str, float]:
        positions, velocities, accelerations, times = self._arrays(result)
        relative = Path("plans") / plan_id / "trajectory.npz"
        destination = self.output_dir / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            destination,
            positions=positions,
            velocities=velocities,
            accelerations=accelerations,
            time_from_start=times,
            joint_names=np.asarray(result.joint_names or [], dtype=np.str_),
        )
        return relative.as_posix(), float(times[-1])

    def record_candidate(
        self,
        generation: int,
        result: TrajectoryPlanResult,
        *,
        start_unix_s: float,
        handoff_unix_s: float | None,
        braking: bool = False,
        reason: str | None = None,
    ) -> str:
        with self._lock:
            plan_id = self._new_plan_id()
            trajectory, duration = self._write_plan(plan_id, result)
            created_s = self._relative_s(time.time_ns())
            plan = _RecordedPlan(
                plan_id=plan_id,
                trajectory=trajectory,
                created_s=created_s,
                start_s=self._relative_s(int(start_unix_s * 1e9)),
                status="braking" if braking else "accepted",
                handoff_s=(
                    None
                    if handoff_unix_s is None
                    else self._relative_s(int(handoff_unix_s * 1e9))
                ),
                duration_s=duration,
                phase_timing=(
                    None
                    if "phase_timing" not in result.diagnostics
                    else {
                        "planning_submitted_s": self._relative_s(
                            int(
                                result.diagnostics["phase_timing"][
                                    "planning_submitted_unix_s"
                                ]
                                * 1e9
                            )
                        ),
                        "bridge_start_s": self._relative_s(
                            int(
                                result.diagnostics["phase_timing"][
                                    "bridge_start_unix_s"
                                ]
                                * 1e9
                            )
                        ),
                        "handoff_s": self._relative_s(
                            int(
                                result.diagnostics["phase_timing"]["handoff_unix_s"]
                                * 1e9
                            )
                        ),
                        "old_continuation_s": float(
                            result.diagnostics["phase_timing"]["old_continuation_s"]
                        ),
                        "bridge_s": float(
                            result.diagnostics["phase_timing"]["bridge_s"]
                        ),
                        "mpd_suffix_s": float(
                            result.diagnostics["phase_timing"]["mpd_suffix_s"]
                        ),
                    }
                ),
            )
            self._plans[plan_id] = plan
            self._generation_to_plan[int(generation)] = plan_id
            if braking:
                self._events.append(
                    {
                        "type": "brake",
                        "time_s": plan.start_s,
                        "duration_s": max(0.2, duration),
                        "plan_id": plan_id,
                        "reason": reason,
                    }
                )
            return plan_id

    def record_rejected(
        self,
        generation: int,
        result: TrajectoryPlanResult,
        *,
        start_unix_s: float,
    ) -> str | None:
        source_value = result.diagnostics.get("trajectory_path")
        if not source_value:
            return None
        source = Path(str(source_value)).expanduser().resolve()
        if not source.is_file():
            return None
        with self._lock:
            plan_id = self._new_plan_id()
            relative = Path("plans") / plan_id / "trajectory.npz"
            destination = self.output_dir / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
            with np.load(destination, allow_pickle=False) as data:
                duration = float(np.asarray(data["time_from_start"])[-1])
            self._plans[plan_id] = _RecordedPlan(
                plan_id=plan_id,
                trajectory=relative.as_posix(),
                created_s=self._relative_s(time.time_ns()),
                start_s=self._relative_s(int(start_unix_s * 1e9)),
                status="rejected",
                duration_s=duration,
            )
            self._generation_to_plan[int(generation)] = plan_id
            return plan_id

    def record_activation(self, generation: int) -> None:
        with self._lock:
            plan_id = self._generation_to_plan.get(int(generation))
            if plan_id is None:
                return
            plan = self._plans[plan_id]
            if self._active_plan_id is not None and self._active_plan_id != plan_id:
                previous = self._plans[self._active_plan_id]
                previous.active_until_s = min(
                    plan.start_s,
                    previous.start_s + previous.duration_s,
                )
                if previous.status == "accepted":
                    previous.status = "superseded"
            plan.active_from_s = plan.start_s
            self._active_plan_id = plan_id
            if plan.handoff_s is not None:
                self._events.append(
                    {"type": "handoff", "time_s": plan.handoff_s, "plan_id": plan_id}
                )

    def record_terminal(self, generation: int, unix_ns: int | None = None) -> None:
        with self._lock:
            plan_id = self._generation_to_plan.get(int(generation))
            if plan_id is None or self._active_plan_id != plan_id:
                return
            plan = self._plans[plan_id]
            terminal_s = self._relative_s(time.time_ns() if unix_ns is None else unix_ns)
            plan.active_until_s = min(
                max(terminal_s, (plan.active_from_s or plan.start_s) + 1e-6),
                plan.start_s + plan.duration_s,
            )
            self._active_plan_id = None

    def _write_manifest_locked(self, unix_ns: int, *, finalize: bool) -> Path:
        duration_s = self._relative_s(unix_ns)
        if finalize:
            for plan in self._plans.values():
                if plan.active_from_s is not None and plan.active_until_s is None:
                    plan.active_until_s = min(
                        max(duration_s, plan.active_from_s + 1e-6),
                        plan.start_s + plan.duration_s,
                    )
                if plan.status == "accepted" and plan.active_from_s is None:
                    plan.status = "rejected"
        duration_s = max(
            duration_s,
            max(
                (
                    plan.active_until_s
                    if plan.active_until_s is not None
                    else min(plan.created_s, duration_s)
                    for plan in self._plans.values()
                ),
                default=0.0,
            ),
            1e-3,
        )
        duration_s = min(duration_s, self._worlds[-1]["valid_until_s"])
        initial_q = self._initial_q
        if initial_q is None:
            first_path = self.output_dir / next(iter(self._plans.values())).trajectory
            with np.load(first_path, allow_pickle=False) as data:
                initial_q = _best_positions_from_archive(data)[0].tolist()
        plans = []
        for plan in self._plans.values():
            payload = {
                "id": plan.plan_id,
                "trajectory": plan.trajectory,
                "created_s": plan.created_s,
                "start_s": plan.start_s,
                "status": plan.status,
            }
            active_until_s = plan.active_until_s
            if plan.active_from_s is not None and active_until_s is None:
                provisional_end = min(duration_s, plan.start_s + plan.duration_s)
                if provisional_end > plan.active_from_s:
                    active_until_s = provisional_end
            if plan.active_from_s is not None and active_until_s is not None:
                payload.update(
                    active_from_s=plan.active_from_s,
                    active_until_s=active_until_s,
                )
            if plan.phase_timing is not None:
                payload["phase_timing"] = plan.phase_timing
            plans.append(payload)
        manifest = {
            "schema": "mpd_dynamic_replay",
            "schema_version": 1,
            "env_name": self.env_name,
            "frame_id": self.frame_id,
            "duration_s": duration_s,
            "initial_q": initial_q,
            "static_scene": self._static_scene,
            "plans": plans,
            "world_snapshots": self._worlds,
            "events": sorted(self._events, key=lambda item: item["time_s"]),
        }
        temporary = self.manifest_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.manifest_path)
        return self.manifest_path

    def flush(self, unix_ns: int | None = None) -> Path | None:
        """Atomically checkpoint a valid manifest without ending the episode."""
        with self._lock:
            if self._closed or self._episode_start_ns is None or not self._worlds or not self._plans:
                return None
            return self._write_manifest_locked(
                time.time_ns() if unix_ns is None else unix_ns,
                finalize=False,
            )

    def close(self, unix_ns: int | None = None) -> Path | None:
        with self._lock:
            if self._closed:
                return self.manifest_path if self.manifest_path.is_file() else None
            if self._episode_start_ns is None or not self._worlds or not self._plans:
                self._closed = True
                return None
            path = self._write_manifest_locked(
                time.time_ns() if unix_ns is None else unix_ns,
                finalize=True,
            )
            self._closed = True
            return path
