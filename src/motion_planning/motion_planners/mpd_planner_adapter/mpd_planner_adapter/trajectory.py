"""Trajectory interpolation used for future-handoff start prediction."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from manipulation_motion_planning.contracts import (
    StartState,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
)


@dataclass(frozen=True)
class TimedPlan:
    result: TrajectoryPlanResult
    start_unix_s: float

    def predict_point(self, unix_s: float) -> TrajectoryPlanPoint:
        if not self.result.valid or not self.result.joint_names or len(self.result.points) < 2:
            raise ValueError("cannot predict from an invalid trajectory")
        relative = max(0.0, unix_s - self.start_unix_s)
        stamps = np.asarray([point.time_from_start_s for point in self.result.points])
        index = int(np.searchsorted(stamps, relative, side="right"))
        index = min(max(index, 1), len(stamps) - 1)
        left, right = self.result.points[index - 1], self.result.points[index]
        width = right.time_from_start_s - left.time_from_start_s
        alpha = 1.0 if relative >= stamps[-1] else (relative - left.time_from_start_s) / width
        alpha = min(1.0, max(0.0, float(alpha)))
        q0, q1 = np.asarray(left.positions), np.asarray(right.positions)
        q = q0 + alpha * (q1 - q0)
        if left.velocities is not None and right.velocities is not None:
            v0, v1 = np.asarray(left.velocities), np.asarray(right.velocities)
            dq = v0 + alpha * (v1 - v0)
        else:
            dq = (q1 - q0) / width
        if left.accelerations is not None and right.accelerations is not None:
            a0, a1 = np.asarray(left.accelerations), np.asarray(right.accelerations)
            ddq = a0 + alpha * (a1 - a0)
        else:
            ddq = (
                np.asarray(right.velocities) - np.asarray(left.velocities)
            ) / width if left.velocities is not None and right.velocities is not None else np.zeros_like(dq)
        if (
            not np.isfinite(q).all()
            or not np.isfinite(dq).all()
            or not np.isfinite(ddq).all()
            or not math.isfinite(unix_s)
        ):
            raise ValueError("predicted state is not finite")
        return TrajectoryPlanPoint(
            positions=q.tolist(),
            velocities=dq.tolist(),
            accelerations=ddq.tolist(),
            time_from_start_s=relative,
        )

    def predict(self, unix_s: float) -> StartState:
        point = self.predict_point(unix_s)
        return StartState(
            joint_names=list(self.result.joint_names),
            positions=list(point.positions),
            velocities=list(point.velocities or []),
            stamp_s=unix_s,
        )
