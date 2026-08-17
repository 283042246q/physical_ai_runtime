from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_defaults_match_production_franka_server() -> None:
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "replan.yaml").read_text(encoding="utf-8")
    )["mpd_replanner"]["ros__parameters"]

    assert config["joint_state_topic"] == "/franka/joint_states"
    assert config["jtc_action_name"] == (
        "/franka_arm_jtc/follow_joint_trajectory"
    )
    assert "target_pose_xyzw" not in config


def test_fake_launch_composes_server_with_owned_jtc_enabled() -> None:
    source = (
        PACKAGE_ROOT / "launch" / "replan_fake_hardware.launch.py"
    ).read_text(encoding="utf-8")

    assert 'FindPackageShare("franka_manipulation_controller_bringup")' in source
    assert '"activate_trajectory_controller": "true"' in source
    assert 'default_value="/franka/joint_states"' in source
    assert 'default_value="/franka_arm_jtc/follow_joint_trajectory"' in source
