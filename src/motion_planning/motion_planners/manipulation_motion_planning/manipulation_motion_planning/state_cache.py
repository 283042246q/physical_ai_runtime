"""Planner-owned robot state cache.

See docs/MOTION_PLANNER_SOURCE_INTERFACE.md Section 3: this is a
passthrough-plus-latest cache, not an observer. It does not filter, fuse, or
predict (Section 3.1) — it keeps the latest `/joint_states` sample, matched by
name, and reports whether that sample is fresh enough to use.
"""

from __future__ import annotations

from typing import Optional

from .contracts import CurrentState


def match_joint_state(
    msg_names: list[str],
    msg_positions: list[float],
    msg_velocities: Optional[list[float]],
    joint_names: list[str],
) -> tuple[Optional[list[float]], Optional[list[float]], list[str]]:
    """Reorder a raw JointState-shaped sample to `joint_names` order.

    Pure function, no ROS/rclpy dependency, so it is unit-testable without a
    node. Returns `(positions, velocities, missing_names)`. `positions` is
    `None` if any configured joint is missing (Section 4: "a reference
    missing any configured joint is dropped", mirroring the EM decoder rule).
    """

    positions_by_name = dict(zip(msg_names, msg_positions))
    velocities_by_name = (
        dict(zip(msg_names, msg_velocities)) if msg_velocities else {}
    )

    missing = [name for name in joint_names if name not in positions_by_name]
    if missing:
        return None, None, missing

    positions = [float(positions_by_name[name]) for name in joint_names]
    velocities = (
        [float(velocities_by_name[name]) for name in joint_names]
        if velocities_by_name and all(name in velocities_by_name for name in joint_names)
        else None
    )
    return positions, velocities, []


class RobotStateCache:
    """Subscribes to a JointState-shaped topic and keeps the latest sample.

    The subscription callback only calls `update()`, which is a cheap
    dict-reorder — safe to run at whatever rate the topic publishes,
    including 500 Hz (Section 2). The planner's solve/tick loop is the only
    caller of `get_fresh()`, on its own schedule (Section 4).

    Freshness is measured against the message's own `header.stamp`
    (`stamp_s`), the same convention EM uses for its `stale_timeout_s` check
    (contracts.md), not wall-clock arrival time. A cache with no header stamp
    available should pass `stamp_s=now_s` at the call site, matching EM's
    "zero stamp -> clock fallback" default.
    """

    def __init__(self, joint_names: list[str], max_age_s: float) -> None:
        if not joint_names:
            raise ValueError("joint_names must not be empty")
        if max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")
        self._joint_names = list(joint_names)
        self._max_age_s = float(max_age_s)
        self._state: Optional[CurrentState] = None
        self._last_missing: list[str] = []

    def update(
        self,
        msg_names: list[str],
        msg_positions: list[float],
        msg_velocities: Optional[list[float]],
        stamp_s: float,
    ) -> None:
        positions, velocities, missing = match_joint_state(
            msg_names, msg_positions, msg_velocities, self._joint_names
        )
        self._last_missing = missing
        if positions is None:
            return
        self._state = CurrentState(
            joint_names=list(self._joint_names),
            positions=positions,
            velocities=velocities,
            stamp_s=stamp_s,
        )

    def get_fresh(self, now_s: float) -> Optional[CurrentState]:
        """Return the latest state if its stamp is within `max_age_s` of `now_s`."""

        if self._state is None:
            return None
        if (now_s - self._state.stamp_s) > self._max_age_s:
            return None
        return self._state

    @property
    def last_missing_joints(self) -> list[str]:
        return list(self._last_missing)
