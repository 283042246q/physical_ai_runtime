from pathlib import Path

import pytest
import yaml

from marvin_motion_planning_bringup.planning_config import load_planning_config


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_default_profile_loads_left_pyroki_mpc() -> None:
    plans = load_planning_config(PACKAGE_ROOT / "config" / "planning.yaml")
    assert len(plans) == 1
    plan = plans[0]
    assert plan.side == "left"
    assert plan.mode == "online_mpc"
    assert plan.backend == "pyroki"
    assert plan.executable == "pyroki_online_mpc_planner"
    assert plan.parameters["source_name"] == "motion_planner_left"
    assert plan.parameters["horizon_steps"] == 5


@pytest.mark.parametrize(
    ("profile", "mode", "executable"),
    [
        ("global_setpoint.yaml", "global_setpoint", "pyroki_global_setpoint_planner"),
        (
            "global_trajectory.yaml",
            "global_trajectory",
            "pyroki_global_trajectory_planner",
        ),
        ("online_mpc.yaml", "online_mpc", "pyroki_online_mpc_planner"),
    ],
)
def test_launch_profiles_select_expected_backend(
    profile: str, mode: str, executable: str
) -> None:
    plans = load_planning_config(PACKAGE_ROOT / "config" / profile)
    assert len(plans) == 1
    assert plans[0].mode == mode
    assert plans[0].executable == executable


def test_button_panel_is_only_in_trajectory_rviz() -> None:
    trajectory = (
        PACKAGE_ROOT / "rviz" / "marvin_global_trajectory.rviz"
    ).read_text(encoding="utf-8")
    streaming = (
        PACKAGE_ROOT / "rviz" / "marvin_streaming_planner.rviz"
    ).read_text(encoding="utf-8")
    assert "rviz_topic_button_panel/EmptyTopicButtonPanel" in trajectory
    assert "rviz_topic_button_panel/EmptyTopicButtonPanel" not in streaming


def test_mpc_marker_waits_for_user_feedback() -> None:
    launch_source = (
        PACKAGE_ROOT / "launch" / "marvin_motion_planning.launch.py"
    ).read_text(encoding="utf-8")
    assert 'plan.mode != "online_mpc"' in launch_source
    assert '"publish_before_first_feedback"' in launch_source


def test_execution_enables_trajectory_goal_contract() -> None:
    execution = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "execution.yaml").read_text(encoding="utf-8")
    )
    for side in ("left", "right"):
        parameters = execution[f"em_{side}"]["ros__parameters"]
        sources = yaml.safe_load(parameters["sources"])
        assert sources[f"motion_planner_{side}"]["goal_contracts"] == [
            "joint_trajectory_goal"
        ]


def test_trajectory_rviz_matches_transient_local_visualization_qos() -> None:
    trajectory = (
        PACKAGE_ROOT / "rviz" / "marvin_global_trajectory.rviz"
    ).read_text(encoding="utf-8")
    assert trajectory.count("Durability Policy: Transient Local") >= 4
    assert "/motion_planning/trajectory_poses_left" in trajectory


def test_arms_can_select_different_modes(tmp_path: Path) -> None:
    config = {
        "arms": {
            "left": {
                "enabled": True,
                "planning": {
                    "mode": "global_setpoint",
                    "backend": {"name": "pyroki", "parameters": {}},
                },
            },
            "right": {
                "enabled": True,
                "planning": {
                    "mode": "global_trajectory",
                    "backend": {
                        "name": "pyroki",
                        "parameters": {"timesteps": 20},
                    },
                },
            },
        }
    }
    path = tmp_path / "planning.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    plans = load_planning_config(path)
    assert [plan.mode for plan in plans] == [
        "global_setpoint",
        "global_trajectory",
    ]
    assert plans[0].parameters["output_joint_names"].startswith("Joint1_L")
    assert plans[1].parameters["timesteps"] == 20


def test_bimanual_group_fails_fast(tmp_path: Path) -> None:
    path = tmp_path / "planning.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "arms": {},
                "groups": {"bimanual": {"enabled": True}},
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="not supported yet"):
        load_planning_config(path)


def test_unsupported_backend_mode_pair_fails(tmp_path: Path) -> None:
    path = tmp_path / "planning.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "arms": {
                    "left": {
                        "enabled": True,
                        "planning": {
                            "mode": "online_mpc",
                            "backend": {"name": "unknown", "parameters": {}},
                        },
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="does not support"):
        load_planning_config(path)
