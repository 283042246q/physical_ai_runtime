"""Configuration dataclasses for PyRoki planner adapters."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PyrokiRobotLoadConfig:
    """Robot loading options shared by PyRoki backends."""

    robot_description_node: str = "robot_state_publisher"
    target_link_name: str = "flange_L"
    load_meshes: bool = False


@dataclass(frozen=True)
class PyrokiGlobalSetpointNodeConfig:
    """ROS-facing configuration for the J-PARSE setpoint source node."""

    pose_topic: str = "/teleop/pose_commands"
    source_name: str = "joint_slider"
    source_namespace: str = "/action_sources"
    command_sink_mode: str = "em"
    direct_command_topic: str = ""
    plan_rate_hz: float = 50.0
    max_state_age_s: float = 0.05
    pose_stale_timeout_s: float = 0.5
    output_joint_names: tuple[str, ...] = ()


@dataclass(frozen=True)
class PyrokiOnlineMpcNodeConfig:
    """ROS-facing configuration for the PyRoki MPC source node."""

    pose_topic: str = "/teleop/pose_commands"
    source_name: str = "pyroki_mpc"
    source_namespace: str = "/action_sources"
    command_sink_mode: str = "em"
    direct_command_topic: str = ""
    step_rate_hz: float = 50.0
    max_state_age_s: float = 0.05
    target_stale_timeout_s: float = 0.1
    joint_names: tuple[str, ...] = ()


@dataclass(frozen=True)
class PyrokiOnlineMpcSolverConfig:
    """Solver-shape options for `solve_online_planning`."""

    horizon_steps: int = 10
