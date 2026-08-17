"""Configuration dataclasses for cuRobo planner adapters."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CuroboRobotConfig:
    """cuRobo robot/scene config names or dictionaries.

    `robot` is normally a cuRobo robot YAML such as `franka.yml`. For project
    robots, this should point at a prepared cuRobo robot config with collision
    spheres, not a raw ROS URDF.
    """

    robot: str = "franka.yml"
    scene_model: str | None = None
    target_link_name: str = ""
    use_cuda_graph: bool = True
    self_collision_check: bool = True
    load_collision_spheres: bool = True


@dataclass(frozen=True)
class CuroboOnlineMpcNodeConfig:
    """ROS-facing configuration for the cuRobo MPC source node."""

    pose_topic: str = "/teleop/pose_commands"
    source_name: str = "curobo_mpc"
    source_namespace: str = "/action_sources"
    command_sink_mode: str = "em"
    direct_command_topic: str = ""
    step_rate_hz: float = 50.0
    max_state_age_s: float = 0.05
    target_stale_timeout_s: float = 0.1
    joint_names: tuple[str, ...] = ()


@dataclass(frozen=True)
class CuroboMpcSolverConfig:
    """Solver-shape options for cuRobo `ModelPredictiveControl`."""

    optimization_dt: float = 0.02
    interpolation_steps: int = 4
    horizon_points: int = 1
    position_tolerance_m: float = 0.005
    orientation_tolerance_rad: float = 0.05
    optimizer_collision_activation_distance_m: float = 0.01
    warm_start_optimization_num_iters: int = 200
    cold_start_optimization_num_iters: int = 300
