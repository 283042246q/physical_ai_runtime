from __future__ import annotations

import math

from .contract import JOINT_NAMES


def trajectory_from_result(result, *, q_start=None, max_start_error_rad=1e-5):
    try:
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    except ImportError as error:
        raise RuntimeError("trajectory_msgs is required to convert a worker result") from error
    names = tuple(result.get("joint_names", ()))
    if names != JOINT_NAMES:
        raise ValueError("worker result has an invalid joint order")
    positions = result.get("positions") or []
    velocities = result.get("velocities") or []
    accelerations = result.get("accelerations") or []
    times = result.get("time_from_start") or []
    if not (len(positions) == len(velocities) == len(accelerations) == len(times)) or len(positions) < 2:
        raise ValueError("worker result must contain positions and matching timestamps")
    if abs(float(times[0])) > 1e-9:
        raise ValueError("trajectory must start at time zero")
    if any(float(right) <= float(left) for left, right in zip(times, times[1:])):
        raise ValueError("trajectory timestamps must be strictly increasing")
    if q_start is not None:
        if len(q_start) != 14:
            raise ValueError("q_start must contain fourteen values")
        error = max(abs(float(a) - float(b)) for a, b in zip(positions[0], q_start))
        if error > float(max_start_error_rad):
            raise ValueError(f"trajectory start error {error:.6g} exceeds limit")
    trajectory = JointTrajectory(joint_names=list(JOINT_NAMES))
    for index, (values, dvalues, ddvalues, stamp) in enumerate(
        zip(positions, velocities, accelerations, times)
    ):
        if len(values) != 14 or len(dvalues) != 14 or len(ddvalues) != 14:
            raise ValueError("each trajectory point must contain fourteen values")
        converted = [float(value) for value in (*values, *dvalues, *ddvalues, stamp)]
        if not all(math.isfinite(value) for value in converted):
            raise ValueError(f"trajectory point {index} contains NaN or Inf")
        point = JointTrajectoryPoint(
            positions=[float(value) for value in values],
            velocities=[float(value) for value in dvalues],
            accelerations=[float(value) for value in ddvalues],
        )
        seconds = int(float(stamp))
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(round((float(stamp) - seconds) * 1e9))
        trajectory.points.append(point)
    return trajectory
