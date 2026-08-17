"""Load and validate the per-arm motion-planning app configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


BACKEND_REGISTRY = {
    ("global_setpoint", "pyroki"): (
        "pyroki_planner_adapter",
        "pyroki_global_setpoint_planner",
    ),
    ("global_trajectory", "pyroki"): (
        "pyroki_planner_adapter",
        "pyroki_global_trajectory_planner",
    ),
    ("online_mpc", "pyroki"): (
        "pyroki_planner_adapter",
        "pyroki_online_mpc_planner",
    ),
}


@dataclass(frozen=True)
class ArmPlanning:
    side: str
    mode: str
    backend: str
    package: str
    executable: str
    parameters: dict[str, Any]


def load_planning_config(path: str | Path) -> list[ArmPlanning]:
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    groups = document.get("groups", {})
    bimanual = groups.get("bimanual", {}) if isinstance(groups, dict) else {}
    if bimanual.get("enabled", False):
        raise ValueError(
            "groups.bimanual is not supported yet; configure independent arms"
        )

    arms = document.get("arms")
    if not isinstance(arms, dict):
        raise ValueError("planning config must contain an 'arms' mapping")

    result: list[ArmPlanning] = []
    for side in ("left", "right"):
        arm = arms.get(side, {})
        if not isinstance(arm, dict):
            raise ValueError(f"arms.{side} must be a mapping")
        if not arm.get("enabled", False):
            continue
        planning = arm.get("planning")
        if not isinstance(planning, dict):
            raise ValueError(f"arms.{side}.planning must be a mapping")
        mode = str(planning.get("mode", ""))
        backend_config = planning.get("backend")
        if not isinstance(backend_config, dict):
            raise ValueError(f"arms.{side}.planning.backend must be a mapping")
        backend = str(backend_config.get("name", ""))
        key = (mode, backend)
        if key not in BACKEND_REGISTRY:
            raise ValueError(
                f"backend '{backend}' does not support planning mode '{mode}'"
            )
        backend_parameters = backend_config.get("parameters", {})
        if not isinstance(backend_parameters, dict):
            raise ValueError(
                f"arms.{side}.planning.backend.parameters must be a mapping"
            )
        target = arm.get("target", {})
        if not isinstance(target, dict):
            raise ValueError(f"arms.{side}.target must be a mapping")

        package, executable = BACKEND_REGISTRY[key]
        parameters = _planner_parameters(side, mode, target, backend_parameters)
        result.append(
            ArmPlanning(side, mode, backend, package, executable, parameters)
        )

    if not result:
        raise ValueError("planning config must enable at least one arm")
    return result


def _planner_parameters(
    side: str,
    mode: str,
    target: dict[str, Any],
    backend_parameters: dict[str, Any],
) -> dict[str, Any]:
    suffix = "L" if side == "left" else "R"
    joint_names = ",".join(f"Joint{index}_{suffix}" for index in range(1, 8))
    parameters: dict[str, Any] = {
        "target_link_name": str(target.get("link_name", f"flange_{suffix}")),
        "robot_description_node": "robot_state_publisher",
        "source_name": f"motion_planner_{side}",
        "source_namespace": "/action_sources",
        "command_sink_mode": "em",
        "pose_topic": str(target.get("pose_topic", f"/teleop/pose_{side}")),
    }
    if mode == "global_setpoint":
        parameters["output_joint_names"] = joint_names
    elif mode == "global_trajectory":
        parameters.update(
            {
                "state_topic": "/joint_states",
                "visualization_topic": f"/motion_planning/trajectory_{side}",
                "visualization_pose_topic": (
                    f"/motion_planning/trajectory_poses_{side}"
                ),
                "visualization_frame": "world",
            }
        )
    elif mode == "online_mpc":
        parameters["joint_names"] = ""
    parameters.update(backend_parameters)
    return parameters
