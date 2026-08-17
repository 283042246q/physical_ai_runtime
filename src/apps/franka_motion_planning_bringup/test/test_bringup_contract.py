from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FR3_JOINTS = [f"fr3_joint{index}" for index in range(1, 8)]
LAUNCH_DIR = PACKAGE_ROOT / "launch"


def _load_config(name: str) -> dict:
    return yaml.safe_load(
        (PACKAGE_ROOT / "config" / name).read_text(encoding="utf-8")
    )


def test_controller_is_jerk_bounded_at_franka_control_rate() -> None:
    config = _load_config("controllers.yaml")
    manager = config["controller_manager"]["ros__parameters"]
    controller = config["franka_arm_jspc"]["ros__parameters"]
    controller_type = config["controller_manager"]["ros__parameters"]["franka_arm_jspc"]["type"]
    behavior = controller["reference_behavior"]

    assert manager["update_rate"] == 1000
    assert controller_type == (
        "manipulation_position_controllers/JointSpaceImpedanceController"
    )
    assert controller["joints"] == FR3_JOINTS
    assert len(controller["kp_stiffness"]) == 7
    assert len(controller["kd_damping"]) == 7
    assert len(controller["max_torques"]) == 7
    assert behavior["mode"] == "ruckig"
    assert behavior["ruckig_control_cycle_s"] == 0.001
    assert behavior["max_velocity_rad_s"] == 2.5
    assert behavior["max_acceleration_rad_s2"] == 10.0
    assert behavior["max_jerk_rad_s3"] == 150.0
    # Stay within FR3 FCI necessary floors for J1–J4 (2.62 / 10 / 5000).
    assert behavior["max_velocity_rad_s"] <= 2.62
    assert behavior["max_acceleration_rad_s2"] <= 10.0
    assert behavior["max_jerk_rad_s3"] < 5000.0


def test_planner_execution_and_controller_joint_contracts_match() -> None:
    planner = _load_config("planner.yaml")["motion_planner"]["ros__parameters"]
    execution = _load_config("execution.yaml")["execution_manager"][
        "ros__parameters"
    ]
    controller = _load_config("controllers.yaml")["franka_arm_jspc"][
        "ros__parameters"
    ]

    planner_joints = planner["output_joint_names"].split(",")
    execution_joints = execution["joint_names"].split(",")
    assert planner_joints == execution_joints == controller["joints"] == FR3_JOINTS
    assert execution["output_topic"] == controller["input_topic"]

    behavior = controller["reference_behavior"]
    assert planner["position_gain"] == 15.0
    assert planner["max_joint_velocity"] == 2.5
    assert planner["max_step_rad"] == pytest.approx(0.05)
    # Keep planner and Ruckig velocity ceilings aligned at the validated profile.
    assert behavior["max_velocity_rad_s"] == planner["max_joint_velocity"]
    assert planner["max_step_rad"] == pytest.approx(
        planner["max_joint_velocity"] / planner["plan_rate_hz"]
    )


def test_marker_starts_at_measured_pose_without_publishing_a_jump() -> None:
    marker = _load_config("marker.yaml")["markers"][0]

    assert marker["base_frame"] == "fr3_link0"
    assert marker["target_frame"] == "fr3_link8"
    assert marker["required_joint_state_topic"] == "/franka/joint_states"
    assert marker["publish_before_first_feedback"] is False


def test_launch_files_split_low_level_and_operator() -> None:
    planning = (LAUNCH_DIR / "planning_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    operator = (LAUNCH_DIR / "operator_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    all_in_one = (LAUNCH_DIR / "franka_motion_planning.launch.py").read_text(
        encoding="utf-8"
    )

    assert "franka_arm_jspc" in planning
    assert "execution_manager" in planning
    assert "pyroki_global_setpoint_planner" not in planning
    assert "rviz_marker_teleop" not in planning

    assert "pyroki_global_setpoint_planner" in operator
    assert "rviz_marker_teleop" in operator
    assert "franka_arm_jspc" not in operator
    assert "franka.launch.py" not in operator

    assert "planning_bringup.launch.py" in all_in_one
    assert "operator_bringup.launch.py" in all_in_one
    assert 'default_value="true"' in all_in_one or "default_value='true'" in (
        all_in_one
    )
    assert "192.168.2.101" in all_in_one
