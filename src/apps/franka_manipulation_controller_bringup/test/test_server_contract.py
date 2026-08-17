from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"


def _load_config(name: str) -> dict:
    return yaml.safe_load((CONFIG_DIR / name).read_text(encoding="utf-8"))


def test_server_configures_exactly_three_effort_routes() -> None:
    config = _load_config("controllers.yaml")
    manager = config["controller_manager"]["ros__parameters"]

    assert manager["franka_arm_jsic"]["type"] == (
        "manipulation_position_controllers/JointSpaceImpedanceController"
    )
    assert manager["franka_arm_tsjic"]["type"] == (
        "manipulation_position_controllers/"
        "TaskSpaceJointImpedanceController"
    )
    assert manager["franka_arm_jtc"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert config["franka_arm_jtc"]["ros__parameters"][
        "command_interfaces"
    ] == ["effort"]


def test_execution_manager_routes_match_controller_contracts() -> None:
    controllers = _load_config("controllers.yaml")
    em = _load_config("execution_manager.yaml")["execution_manager"][
        "ros__parameters"
    ]

    assert em["output_topic"] == controllers["franka_arm_jsic"][
        "ros__parameters"
    ]["input_topic"]
    assert em["pose_output_topic"] == controllers["franka_arm_tsjic"][
        "ros__parameters"
    ]["pose_topic"]
    assert em["pose_chunk_output_topic"] == controllers["franka_arm_tsjic"][
        "ros__parameters"
    ]["pose_chunk_topic"]
    assert em["twist_output_topic"] == controllers["franka_arm_tsjic"][
        "ros__parameters"
    ]["twist_topic"]
    assert em["jtc_action_name"] == (
        "/franka_arm_jtc/follow_joint_trajectory"
    )


def test_manual_source_has_highest_priority_and_full_task_space_contract() -> None:
    em = _load_config("execution_manager.yaml")["execution_manager"][
        "ros__parameters"
    ]
    sources = yaml.safe_load(em["sources"])

    assert sources["marker"]["priority"] > sources["trajectory_test"][
        "priority"
    ]
    assert sources["trajectory_test"]["priority"] > sources[
        "motion_planner"
    ]["priority"]
    assert sources["motion_planner"]["priority"] > sources["policy"][
        "priority"
    ]
    assert sources["marker"]["pose_contracts"] == [
        "pose_target",
        "pose_chunk",
    ]
    assert sources["marker"]["twist_contracts"] == ["twist_target"]
    assert sources["trajectory_test"]["goal_contracts"] == [
        "joint_trajectory_goal"
    ]


def test_em_starts_only_after_route_spawner_succeeds() -> None:
    source = (
        PACKAGE_ROOT / "launch" / "controller_bringup.launch.py"
    ).read_text(encoding="utf-8")

    assert "'franka_arm_tsjic'" in source
    assert "'franka_arm_jsic'" in source
    assert "'franka_arm_jtc'" in source
    assert "'--inactive'" in source
    assert "target_action=inactive_controller_spawner" in source
    assert "target_action=trajectory_controller_spawner" in source
    assert "[execution_manager]" in source
    assert "Shutdown(reason=reason)" in source


def test_external_trajectory_owner_is_explicit_opt_in() -> None:
    source = (
        PACKAGE_ROOT / "launch" / "controller_bringup.launch.py"
    ).read_text(encoding="utf-8")

    assert "'activate_trajectory_controller'" in source
    assert "default_value='false'" in source
    assert "'franka_arm_jtc',\n                '--controller-manager'" in source
