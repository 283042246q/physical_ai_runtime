from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def test_phase4_has_separate_launch_socket_and_node_names():
    launch = (PACKAGE / "launch" / "replan_dynamic.launch.py").read_text()
    config = (PACKAGE / "config" / "replan_dynamic.yaml").read_text()
    assert "mpd_dynamic_replanner" in launch
    assert "mpd-dynamic-runtime.sock" in config
    assert "trajectory_duration_s: 10.0" in config
    assert "clearance_score_mode: mean_cvar" in config
    assert "clearance_cvar_fraction: 0.10" in config
    assert "clearance_mean_weight: 0.25" in config
    assert "clearance_cvar_weight: 0.75" in config
    assert "dynamic_replan_node" in launch
    assert "jtc_safe_stop" in launch


def test_fake_launch_uses_franka_server_fake_hardware_and_jtc():
    launch = (PACKAGE / "launch" / "replan_dynamic_fake_hardware.launch.py").read_text()
    assert "franka_manipulation_controller_bringup" in launch
    assert '"use_fake_hardware": "true"' in launch
    assert '"activate_trajectory_controller": "true"' in launch
    assert "dynamic_world_demo" in launch
    assert "world_scenario_file" in launch
    assert "planner_seed" in launch
    assert '"config": LaunchConfiguration("config")' in launch


def test_phase4_aligned_profile_matches_requested_ros_weights():
    document = yaml.safe_load(
        (PACKAGE / "config" / "replan_dynamic_aligned.yaml").read_text()
    )
    parameters = document["mpd_dynamic_replanner"]["ros__parameters"]

    assert parameters["expected_phase4_aligned"] is True
    assert parameters["socket_path"].endswith(
        "mpd-dynamic-aligned-runtime.sock"
    )
    assert parameters["cost_deviation_weight"] == 0.15
    assert parameters["switching_hysteresis"] == 0.02
    assert parameters["relative_switching_hysteresis"] == 0.10
    assert parameters["split_terminal_hold_clearance"] is True
    assert parameters["cost_motion_clearance_weight"] == 4.0
    assert parameters["cost_terminal_hold_clearance_weight"] == 0.5
    assert parameters["comparison_horizon_s"] == 2.0
    assert parameters["trajectory_duration_s"] == 10.0
    assert parameters["cost_kinematic_weight"] == 1.0
    assert parameters["cost_tail_kinematic_weight"] == 3.0
    assert parameters["adaptive_deviation_weight_enabled"] is True


def test_phase5_has_separate_entry_config_socket_and_mode():
    launch = (PACKAGE / "launch" / "replan_space_time.launch.py").read_text()
    config = (PACKAGE / "config" / "replan_space_time.yaml").read_text()
    assert "space_time_replan_node" in launch
    assert "mpd_space_time_replanner" in launch
    assert "mpd-space-time-runtime.sock" in config
    assert "phase5_joint" in launch
    assert "cost_tail_kinematic_weight: 3.0" in config
    assert "cost_motion_clearance_weight: 4.0" in config
    assert "cost_terminal_hold_clearance_weight: 0.5" in config
    assert "adaptive_deviation_weight_enabled: true" in config
    assert "cost_deviation_weight: 0.15" in config
    assert "relative_switching_hysteresis: 0.10" in config
    assert "jtc_safe_stop" in launch


def test_phase5_fake_launch_keeps_franka_fake_hardware_path():
    launch = (PACKAGE / "launch" / "replan_space_time_fake_hardware.launch.py").read_text()
    assert "franka_manipulation_controller_bringup" in launch
    assert '"use_fake_hardware": "true"' in launch
    assert '"activate_trajectory_controller": "true"' in launch
    assert "replan_space_time.launch.py" in launch
    assert "dynamic_world_demo" in launch
    assert "world_scenario_file" in launch
    assert "planner_seed" in launch
