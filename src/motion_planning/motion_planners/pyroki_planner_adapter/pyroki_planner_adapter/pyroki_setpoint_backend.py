"""PyRoki J-PARSE setpoint backend (`GlobalSetpointBackend`)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import _bootstrap  # noqa: F401

import pyroki as pk
from manipulation_motion_planning.contracts import (
    JointTarget,
    PoseTarget,
    SetpointPlanResult,
    StartState,
    Target,
    World,
)
from manipulation_motion_planning.state_cache import match_joint_state

from ._jparse import jparse_step


def _scalar_diagnostics(info: dict) -> dict:
    keys = (
        "position_error",
        "orientation_error",
        "max_joint_vel",
        "manipulability",
        "inverse_condition_number",
    )
    return {key: float(info[key]) for key in keys if key in info}


@dataclass(frozen=True)
class JparseSolverConfig:
    """Tunable J-PARSE / velocity IK parameters for global setpoint solving."""

    method: str = "jparse"
    gamma: float = 0.1
    singular_direction_gain_position: float = 1.0
    singular_direction_gain_angular: float = 1.0
    position_gain: float = 5.0
    orientation_gain: float = 1.0
    nullspace_gain: float = 0.5
    max_joint_velocity: float = 2.0
    dls_damping: float = 0.05

    def as_jparse_kwargs(self) -> dict:
        return {
            "method": self.method,
            "gamma": self.gamma,
            "singular_direction_gain_position": self.singular_direction_gain_position,
            "singular_direction_gain_angular": self.singular_direction_gain_angular,
            "position_gain": self.position_gain,
            "orientation_gain": self.orientation_gain,
            "nullspace_gain": self.nullspace_gain,
            "max_joint_velocity": self.max_joint_velocity,
            "dls_damping": self.dls_damping,
        }


class PyrokiJparseSetpointBackend:
    """Global setpoint IK via iterative J-PARSE steps to a pose goal."""

    def __init__(
        self,
        robot: pk.Robot,
        target_link_name: str,
        *,
        solver: JparseSolverConfig | None = None,
        home_cfg: np.ndarray | None = None,
    ) -> None:
        self._robot = robot
        self._actuated_names = list(robot.joints.actuated_names)
        try:
            self._target_link_index = robot.links.names.index(target_link_name)
        except ValueError as exc:
            raise ValueError(
                f"Target link '{target_link_name}' not in URDF links: {robot.links.names}"
            ) from exc
        self._solver = solver or JparseSolverConfig()
        if self._solver.method not in {"jparse", "pinv", "dls"}:
            raise ValueError(
                f"Unsupported IK method '{self._solver.method}'. "
                "Expected one of: jparse, pinv, dls."
            )
        limits_mid = (robot.joints.lower_limits + robot.joints.upper_limits) / 2.0
        self._home_cfg = (
            np.asarray(home_cfg, dtype=np.float64)
            if home_cfg is not None
            else np.asarray(limits_mid, dtype=np.float64)
        )

    def warmup(self) -> None:
        cfg = self._home_cfg.copy()
        jparse_step(
            robot=self._robot,
            cfg=cfg,
            target_link_index=self._target_link_index,
            target_position=np.array([0.2, 0.0, 0.2], dtype=np.float32),
            target_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            **self._solver.as_jparse_kwargs(),
            dt=0.02,
            home_cfg=self._home_cfg,
        )

    def update_world(self, world: Optional[World]) -> None:
        del world

    def plan(
        self, start_state: StartState, target: Target, options: dict
    ) -> SetpointPlanResult:
        if isinstance(target, JointTarget):
            return self._plan_joint_target(target)
        if not isinstance(target, PoseTarget):
            return SetpointPlanResult(
                valid=False,
                reason=f"unsupported target type: {type(target).__name__}",
            )
        return self._plan_pose_target(start_state, target, options)

    def _plan_joint_target(self, target: JointTarget) -> SetpointPlanResult:
        missing = [name for name in target.joint_names if name not in self._actuated_names]
        if missing:
            return SetpointPlanResult(
                valid=False,
                reason=f"joint target names not in robot model: {missing}",
            )
        if len(target.joint_names) != len(target.positions):
            return SetpointPlanResult(
                valid=False,
                reason="joint_names and positions length mismatch",
            )
        return SetpointPlanResult(
            valid=True,
            joint_names=list(target.joint_names),
            positions=[float(p) for p in target.positions],
        )

    def _plan_pose_target(
        self, start_state: StartState, target: PoseTarget, options: dict
    ) -> SetpointPlanResult:
        positions, _, missing = match_joint_state(
            start_state.joint_names,
            start_state.positions,
            start_state.velocities,
            self._actuated_names,
        )
        if positions is None:
            return SetpointPlanResult(
                valid=False,
                reason=f"start state missing joints: {missing}",
            )

        max_iterations = int(options.get("max_iterations", 200))
        dt = float(options.get("dt", 0.02))
        position_tol_m = float(options.get("position_tolerance_m", 1e-3))
        orientation_tol_rad = float(options.get("orientation_tolerance_rad", 1e-2))
        max_step_rad = float(options.get("max_step_rad", 0.05))
        require_convergence = bool(options.get("require_convergence", False))

        cfg = np.asarray(positions, dtype=np.float64)
        target_position = np.asarray(target.position_xyz, dtype=np.float32)
        target_wxyz = np.asarray(target.orientation_wxyz, dtype=np.float32)

        last_info: dict = {}
        converged = False
        for iteration in range(max_iterations):
            next_cfg, info = jparse_step(
                robot=self._robot,
                cfg=cfg,
                target_link_index=self._target_link_index,
                target_position=target_position,
                target_wxyz=target_wxyz,
                **self._solver.as_jparse_kwargs(),
                dt=dt,
                home_cfg=self._home_cfg,
            )
            delta = np.clip(next_cfg - cfg, -max_step_rad, max_step_rad)
            cfg = cfg + delta
            last_info = info

            pos_err = float(info.get("position_error", float("inf")))
            ori_err = float(info.get("orientation_error", 0.0))
            if pos_err <= position_tol_m and ori_err <= orientation_tol_rad:
                converged = True
                break

        if not all(np.isfinite(cfg)):
            return SetpointPlanResult(
                valid=False,
                reason="non_finite joint configuration from J-PARSE",
                diagnostics={"iterations": iteration + 1, **_scalar_diagnostics(last_info)},
            )

        if require_convergence and not converged:
            return SetpointPlanResult(
                valid=False,
                reason=(
                    f"J-PARSE did not converge within {max_iterations} iterations "
                    f"(position_error={last_info.get('position_error')}, "
                    f"orientation_error={last_info.get('orientation_error')})"
                ),
                diagnostics={
                    "iterations": max_iterations,
                    "converged": False,
                    **_scalar_diagnostics(last_info),
                },
            )

        return SetpointPlanResult(
            valid=True,
            joint_names=list(self._actuated_names),
            positions=cfg.tolist(),
            diagnostics={
                "iterations": iteration + 1,
                "converged": converged,
                **_scalar_diagnostics(last_info),
            },
        )
