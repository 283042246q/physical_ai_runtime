"""PyRoki gradient-based horizon MPC backend adapter shell."""

from __future__ import annotations

from typing import Optional

from . import _bootstrap  # noqa: F401

import numpy as np
import pyroki as pk
from manipulation_motion_planning.contracts import (
    CurrentState,
    HorizonPlanResult,
    Target,
    World,
)

from .config import PyrokiOnlineMpcSolverConfig
from .world_adapter import PyrokiWorldAdapter

_NOT_IMPLEMENTED_MSG = (
    "pyroki_planner_adapter: PyrokiHorizonMpcBackend is not implemented yet. "
    "See docs/MOTION_PLANNER_SOURCE_INTERFACE.md Section 6.2."
)


class PyrokiHorizonMpcBackend:
    """`OnlineMpcBackend` for PyRoki solve_online_planning (gradient MPC)."""

    def __init__(
        self,
        robot: pk.Robot,
        target_link_name: str,
        *,
        robot_collision: pk.collision.RobotCollision | None = None,
        solver_config: PyrokiOnlineMpcSolverConfig | None = None,
        world_adapter: PyrokiWorldAdapter | None = None,
    ) -> None:
        self._robot = robot
        self._target_link_name = target_link_name
        if target_link_name not in robot.links.names:
            raise ValueError(
                f"Target link '{target_link_name}' not in URDF links: {robot.links.names}"
            )
        self._robot_collision = robot_collision
        self._solver_config = solver_config or PyrokiOnlineMpcSolverConfig()
        self._world_adapter = world_adapter or PyrokiWorldAdapter()
        self._world_collision: list[pk.collision.CollGeom] = []
        self._target: Target | None = None
        self._prev_sols: np.ndarray | None = None

    def warmup(self) -> None:
        """Reserved for JAX analysis/JIT once `solve_online_planning` is wired."""

    def reset(self, current_state: CurrentState) -> None:
        positions = np.asarray(current_state.positions, dtype=np.float64)
        self._prev_sols = np.repeat(
            positions[None, :],
            self._solver_config.horizon_steps,
            axis=0,
        )

    def update_target(self, target: Target) -> None:
        self._target = target

    def update_world(self, world: Optional[World]) -> None:
        self._world_collision = self._world_adapter.update(world)

    def step(self, current_state: CurrentState, dt: float) -> HorizonPlanResult:
        del current_state, dt
        raise NotImplementedError(_NOT_IMPLEMENTED_MSG)
