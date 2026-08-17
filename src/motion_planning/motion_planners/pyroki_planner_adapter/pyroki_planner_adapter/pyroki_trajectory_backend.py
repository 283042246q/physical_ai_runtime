"""PyRoki trajopt trajectory backend (`GlobalTrajectoryBackend`) adapter shell."""

from __future__ import annotations

from typing import Optional

from . import _bootstrap  # noqa: F401

import pyroki as pk
from manipulation_motion_planning.contracts import (
    StartState,
    Target,
    TrajectoryPlanResult,
    World,
)

from .world_adapter import PyrokiWorldAdapter


_NOT_IMPLEMENTED_MSG = (
    "pyroki_planner_adapter: PyrokiTrajoptTrajectoryBackend is not implemented yet. "
    "See upstream PyRoki examples/07_trajopt.py."
)


class PyrokiTrajoptTrajectoryBackend:
    """Global trajectory planning via PyRoki `solve_trajopt`."""

    def __init__(
        self,
        robot: pk.Robot,
        target_link_name: str,
        *,
        robot_collision: pk.collision.RobotCollision | None = None,
        world_adapter: PyrokiWorldAdapter | None = None,
    ) -> None:
        self._robot = robot
        self._target_link_name = target_link_name
        if target_link_name not in robot.links.names:
            raise ValueError(
                f"Target link '{target_link_name}' not in URDF links: {robot.links.names}"
            )
        self._robot_collision = robot_collision
        self._world_adapter = world_adapter or PyrokiWorldAdapter()
        self._world_collision: list[pk.collision.CollGeom] = []

    def warmup(self) -> None:
        """Reserved for JAX analysis/JIT once `solve_trajopt` is wired."""

    def update_world(self, world: Optional[World]) -> None:
        self._world_collision = self._world_adapter.update(world)

    def plan(
        self, start_state: StartState, target: Target, options: dict
    ) -> TrajectoryPlanResult:
        del start_state, target, options
        raise NotImplementedError(_NOT_IMPLEMENTED_MSG)
