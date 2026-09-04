from __future__ import annotations

from .contract import JOINT_NAMES


def trajectory_from_result(result):
    try:
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    except ImportError as error:
        raise RuntimeError("trajectory_msgs is required to convert a worker result") from error
    names = tuple(result.get("joint_names", ()))
    if names != JOINT_NAMES:
        raise ValueError("worker result has an invalid joint order")
    positions = result.get("positions") or []
    times = result.get("time_from_start") or []
    if len(positions) != len(times) or len(positions) < 2:
        raise ValueError("worker result must contain positions and matching timestamps")
    if any(float(right) <= float(left) for left, right in zip(times, times[1:])):
        raise ValueError("trajectory timestamps must be strictly increasing")
    trajectory = JointTrajectory(joint_names=list(JOINT_NAMES))
    for values, stamp in zip(positions, times):
        if len(values) != 14:
            raise ValueError("each trajectory point must contain fourteen positions")
        point = JointTrajectoryPoint(positions=[float(value) for value in values])
        seconds = int(float(stamp))
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(round((float(stamp) - seconds) * 1e9))
        trajectory.points.append(point)
    return trajectory
