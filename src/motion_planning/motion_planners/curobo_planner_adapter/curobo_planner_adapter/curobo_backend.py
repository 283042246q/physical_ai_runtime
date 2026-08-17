"""cuRobo backend adapters for the planner-source contracts."""

from __future__ import annotations

from typing import Optional

from . import _bootstrap  # noqa: F401

import torch
from curobo.model_predictive_control import (
    ModelPredictiveControl,
    ModelPredictiveControlCfg,
)
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose, JointState, Pose
from manipulation_motion_planning.contracts import (
    CurrentState,
    HorizonPlanPoint,
    HorizonPlanResult,
    JointTarget,
    PoseTarget,
    StartState,
    Target,
    TrajectoryPlanPoint,
    TrajectoryPlanResult,
    World,
)

from .config import CuroboMpcSolverConfig, CuroboRobotConfig
from .world_adapter import CuroboWorldAdapter


class CuroboMotionPlannerBackend:
    """`GlobalTrajectoryBackend` using cuRobo `MotionPlanner`."""

    def __init__(
        self,
        robot_config: CuroboRobotConfig | None = None,
        *,
        world_adapter: CuroboWorldAdapter | None = None,
    ) -> None:
        self._robot_config = robot_config or CuroboRobotConfig()
        self._world_adapter = world_adapter or CuroboWorldAdapter()
        self._scene = None
        self._planner = self._create_planner()

    @property
    def joint_names(self) -> list[str]:
        return list(self._planner.joint_names)

    @property
    def tool_frames(self) -> list[str]:
        return list(self._planner.tool_frames)

    def warmup(self) -> None:
        self._planner.warmup(enable_graph=self._robot_config.use_cuda_graph)

    def update_world(self, world: Optional[World]) -> None:
        self._scene = self._world_adapter.update(world)
        if self._scene is not None:
            self._planner.update_world(self._scene)

    def plan(
        self, start_state: StartState, target: Target, options: dict
    ) -> TrajectoryPlanResult:
        current_state = _joint_state_from_contract(start_state, self.joint_names)
        if isinstance(target, JointTarget):
            return self._plan_joint_target(current_state, target, options)
        if not isinstance(target, PoseTarget):
            return TrajectoryPlanResult(valid=False, reason="unsupported_target")

        goal = _goal_tool_pose_from_target(
            target,
            target_link_name=_target_link_name(
                self._robot_config.target_link_name,
                self.tool_frames,
            ),
            tool_frames=self.tool_frames,
        )
        result = self._planner.plan_pose(
            goal,
            current_state,
            max_attempts=int(options.get("max_attempts", 5)),
            enable_graph_attempt=int(options.get("enable_graph_attempt", 1)),
        )
        if result is None or not _success_any(result.success):
            return TrajectoryPlanResult(valid=False, reason="curobo_plan_failed")
        interp = result.get_interpolated_plan()
        dt = float(
            options.get(
                "dt",
                getattr(self._planner.trajopt_solver.config, "interpolation_dt", 0.02),
            )
        )
        return TrajectoryPlanResult(
            valid=True,
            joint_names=self.joint_names,
            points=_trajectory_points_from_joint_state(interp, dt),
            diagnostics={"backend": "curobo_motion_planner"},
        )

    def _create_planner(self) -> MotionPlanner:
        scene_model = self._scene or self._robot_config.scene_model
        cfg = MotionPlannerCfg.create(
            robot=self._robot_config.robot,
            scene_model=scene_model,
            use_cuda_graph=self._robot_config.use_cuda_graph,
            self_collision_check=self._robot_config.self_collision_check,
        )
        return MotionPlanner(cfg)

    def _plan_joint_target(
        self,
        current_state: JointState,
        target: JointTarget,
        options: dict,
    ) -> TrajectoryPlanResult:
        goal_position = _reordered_positions(
            target.joint_names,
            target.positions,
            self.joint_names,
        )
        goal_state = JointState.from_position(
            torch.tensor([goal_position], device="cuda", dtype=torch.float32),
            joint_names=self.joint_names,
        )
        result = self._planner.plan_cspace(
            goal_state,
            current_state,
            max_attempts=int(options.get("max_attempts", 5)),
        )
        if result is None or not _success_any(result.success):
            return TrajectoryPlanResult(valid=False, reason="curobo_cspace_plan_failed")
        dt = float(
            options.get(
                "dt",
                getattr(self._planner.trajopt_solver.config, "interpolation_dt", 0.02),
            )
        )
        return TrajectoryPlanResult(
            valid=True,
            joint_names=self.joint_names,
            points=_trajectory_points_from_joint_state(result.get_interpolated_plan(), dt),
            diagnostics={"backend": "curobo_motion_planner"},
        )


class CuroboMpcBackend:
    """`OnlineMpcBackend` using cuRobo `ModelPredictiveControl`."""

    def __init__(
        self,
        robot_config: CuroboRobotConfig | None = None,
        solver_config: CuroboMpcSolverConfig | None = None,
        *,
        world_adapter: CuroboWorldAdapter | None = None,
    ) -> None:
        self._robot_config = robot_config or CuroboRobotConfig()
        self._solver_config = solver_config or CuroboMpcSolverConfig()
        self._world_adapter = world_adapter or CuroboWorldAdapter()
        self._scene = None
        self._solver = self._create_solver()
        self._target: Target | None = None
        self._is_setup = False

    @property
    def joint_names(self) -> list[str]:
        return list(self._solver.joint_names)

    @property
    def tool_frames(self) -> list[str]:
        return list(self._solver.tool_frames)

    def warmup(self) -> None:
        """cuRobo compiles the main graph after `setup()` has a current state."""

    def reset(self, current_state: CurrentState) -> None:
        state = _joint_state_from_contract(current_state, self.joint_names)
        self._solver.setup(state)
        self._is_setup = True

    def update_target(self, target: Target) -> None:
        self._target = target

    def update_world(self, world: Optional[World]) -> None:
        self._scene = self._world_adapter.update(world)

    def step(self, current_state: CurrentState, dt: float) -> HorizonPlanResult:
        if self._target is None:
            return HorizonPlanResult(valid=False, reason="target_missing")
        if isinstance(self._target, JointTarget):
            return HorizonPlanResult(
                valid=False,
                reason="joint_target_tracking_not_wired_for_curobo_mpc",
            )
        if not isinstance(self._target, PoseTarget):
            return HorizonPlanResult(valid=False, reason="unsupported_target")

        state = _joint_state_from_contract(current_state, self.joint_names)
        if not self._is_setup:
            self._solver.setup(state)
            self._is_setup = True

        goal = _goal_tool_pose_from_target(
            self._target,
            target_link_name=_target_link_name(
                self._robot_config.target_link_name,
                self.tool_frames,
            ),
            tool_frames=self.tool_frames,
        )
        goal_updated = self._solver.update_goal_tool_poses(goal, run_ik=True)
        if not goal_updated:
            return HorizonPlanResult(valid=False, reason="curobo_goal_update_failed")

        result = self._solver.optimize_action_sequence(state)
        action_sequence = getattr(result, "action_sequence", None)
        if action_sequence is None or action_sequence.position is None:
            return HorizonPlanResult(valid=False, reason="curobo_mpc_no_action_sequence")

        points = _horizon_points_from_action_sequence(
            action_sequence,
            dt=dt,
            horizon_points=max(1, int(self._solver_config.horizon_points)),
        )
        if not points:
            return HorizonPlanResult(valid=False, reason="curobo_mpc_empty_horizon")
        return HorizonPlanResult(
            valid=True,
            points=points,
            diagnostics={"backend": "curobo_mpc"},
        )

    def _create_solver(self) -> ModelPredictiveControl:
        scene_model = self._scene or self._robot_config.scene_model
        cfg = ModelPredictiveControlCfg.create(
            robot=self._robot_config.robot,
            scene_model=scene_model,
            use_cuda_graph=self._robot_config.use_cuda_graph,
            self_collision_check=self._robot_config.self_collision_check,
            load_collision_spheres=self._robot_config.load_collision_spheres,
            optimization_dt=self._solver_config.optimization_dt,
            interpolation_steps=self._solver_config.interpolation_steps,
            position_tolerance=self._solver_config.position_tolerance_m,
            orientation_tolerance=self._solver_config.orientation_tolerance_rad,
            optimizer_collision_activation_distance=(
                self._solver_config.optimizer_collision_activation_distance_m
            ),
            warm_start_optimization_num_iters=(
                self._solver_config.warm_start_optimization_num_iters
            ),
            cold_start_optimization_num_iters=(
                self._solver_config.cold_start_optimization_num_iters
            ),
        )
        return ModelPredictiveControl(cfg)


def _joint_state_from_contract(state: CurrentState, joint_names: list[str]) -> JointState:
    positions = _reordered_positions(state.joint_names, state.positions, joint_names)
    joint_state = JointState.from_position(
        torch.tensor([positions], device="cuda", dtype=torch.float32),
        joint_names=joint_names,
    )
    if state.velocities is not None:
        velocities = _reordered_positions(state.joint_names, state.velocities, joint_names)
        joint_state.velocity = torch.tensor(
            [velocities], device="cuda", dtype=torch.float32
        )
    else:
        joint_state.velocity = torch.zeros_like(joint_state.position)
    joint_state.acceleration = torch.zeros_like(joint_state.position)
    return joint_state


def _goal_tool_pose_from_target(
    target: PoseTarget,
    *,
    target_link_name: str,
    tool_frames: list[str],
) -> GoalToolPose:
    pose = Pose(
        position=torch.tensor(
            [list(target.position_xyz)],
            device="cuda",
            dtype=torch.float32,
        ),
        quaternion=torch.tensor(
            [list(target.orientation_wxyz)],
            device="cuda",
            dtype=torch.float32,
        ),
        name=target_link_name,
    )
    return GoalToolPose.from_poses(
        {target_link_name: pose},
        ordered_tool_frames=tool_frames,
        num_goalset=1,
    )


def _target_link_name(configured: str, tool_frames: list[str]) -> str:
    if configured:
        if configured not in tool_frames:
            raise ValueError(
                f"target_link_name {configured!r} not in cuRobo tool_frames {tool_frames}"
            )
        return configured
    if not tool_frames:
        raise ValueError("cuRobo config has no tool_frames")
    return tool_frames[0]


def _reordered_positions(
    input_joint_names: list[str],
    positions: list[float],
    output_joint_names: list[str],
) -> list[float]:
    by_name = dict(zip(input_joint_names, positions))
    missing = [name for name in output_joint_names if name not in by_name]
    if missing:
        raise ValueError(f"state missing cuRobo joints: {missing}")
    return [float(by_name[name]) for name in output_joint_names]


def _success_any(success) -> bool:  # noqa: ANN001
    if isinstance(success, bool):
        return success
    if hasattr(success, "any"):
        return bool(success.any().item())
    return bool(success)


def _trajectory_points_from_joint_state(
    joint_state: JointState,
    dt: float,
) -> list[TrajectoryPlanPoint]:
    positions = joint_state.position.detach().cpu()
    velocities = getattr(joint_state, "velocity", None)
    velocities_cpu = velocities.detach().cpu() if velocities is not None else None
    if positions.ndim == 3:
        positions = positions[0]
        if velocities_cpu is not None:
            velocities_cpu = velocities_cpu[0]
    return [
        TrajectoryPlanPoint(
            positions=[float(v) for v in positions[index].tolist()],
            velocities=(
                [float(v) for v in velocities_cpu[index].tolist()]
                if velocities_cpu is not None
                else None
            ),
            time_from_start_s=float(index) * dt,
        )
        for index in range(positions.shape[0])
    ]


def _horizon_points_from_action_sequence(
    action_sequence: JointState,
    *,
    dt: float,
    horizon_points: int,
) -> list[HorizonPlanPoint]:
    positions = action_sequence.position.detach().cpu()
    velocities = getattr(action_sequence, "velocity", None)
    velocities_cpu = velocities.detach().cpu() if velocities is not None else None
    if positions.ndim == 3:
        positions = positions[0]
        if velocities_cpu is not None:
            velocities_cpu = velocities_cpu[0]
    if positions.ndim != 2 or positions.shape[0] == 0:
        return []

    if horizon_points == 1:
        indices = [positions.shape[0] - 1]
    else:
        indices = list(range(min(horizon_points, positions.shape[0])))

    points: list[HorizonPlanPoint] = []
    for output_index, source_index in enumerate(indices):
        points.append(
            HorizonPlanPoint(
                positions=[float(v) for v in positions[source_index].tolist()],
                velocities=(
                    [float(v) for v in velocities_cpu[source_index].tolist()]
                    if velocities_cpu is not None
                    else None
                ),
                time_from_start_s=float(output_index) * dt,
            )
        )
    return points
