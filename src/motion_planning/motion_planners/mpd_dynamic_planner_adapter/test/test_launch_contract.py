from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_phase4_has_separate_launch_socket_and_node_names():
    launch = (PACKAGE / "launch" / "replan_dynamic.launch.py").read_text()
    config = (PACKAGE / "config" / "replan_dynamic.yaml").read_text()
    assert "mpd_dynamic_replanner" in launch
    assert "mpd-dynamic-runtime.sock" in config
    assert "dynamic_replan_node" in launch
    assert "jtc_safe_stop" in launch


def test_fake_launch_uses_franka_server_fake_hardware_and_jtc():
    launch = (PACKAGE / "launch" / "replan_dynamic_fake_hardware.launch.py").read_text()
    assert "franka_manipulation_controller_bringup" in launch
    assert '"use_fake_hardware": "true"' in launch
    assert '"activate_trajectory_controller": "true"' in launch
    assert "dynamic_world_demo" in launch
