import runpy
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_package_has_four_launch_demos() -> None:
    launches = sorted(
        path.name for path in (PACKAGE_ROOT / "launch").glob("*.launch.py")
    )
    assert launches == [
        "global_ik_demo.launch.py",
        "jtc_probe_demo.launch.py",
        "move_to_start_demo.launch.py",
        "task_space_marker_demo.launch.py",
    ]


def test_global_ik_routes_setpoints_to_server_em() -> None:
    source = _read("launch/global_ik_demo.launch.py")
    assert 'executable="pyroki_global_setpoint_planner"' in source
    assert '"source_name": "motion_planner"' in source
    assert '"command_sink_mode": "em"' in source
    assert '"publish_before_first_feedback": False' in source
    assert "controller_manager" not in source
    assert "manipulation_execution_manager" not in source


def test_task_space_marker_routes_pose_directly_to_em() -> None:
    source = _read("launch/task_space_marker_demo.launch.py")
    assert "/action_sources/marker/arm/cartesian_pose" in source
    assert '"publish_before_first_feedback": False' in source
    assert "pyroki" not in source


def test_jtc_probe_uses_previous_smooth_trajectory_demo() -> None:
    source = _read("scripts/smooth_trajectory.py")
    launch = _read("launch/jtc_probe_demo.launch.py")
    assert '"amplitude_rad", 0.25' in source
    assert '"duration_s", 4.0' in source
    assert '"num_points", 80' in source
    assert "/action_sources/trajectory_test/arm/joint_trajectory" in source
    assert "ActionClient" in source
    assert "get_result_async" in source
    assert 'executable="smooth_trajectory.py"' in launch
    assert "controller_manager" not in launch
    assert "manipulation_execution_manager" not in launch


def test_mpd_trajectory_uses_em_action_and_safe_default() -> None:
    source = _read("scripts/send_mpd_trajectory.py")
    cmake = _read("CMakeLists.txt")
    assert "/action_sources/trajectory_test/arm/joint_trajectory" in source
    assert "ActionClient" in source
    assert "FollowJointTrajectory" in source
    assert "send_goal_async" in source
    assert "get_result_async" in source
    assert 'self.declare_parameter("plan_only", True)' in source
    assert "TARGET_CARTESIAN_POSE_XYZW=" in source
    assert "BEST_TRAJECTORY_TERMINAL_CARTESIAN_POSE_XYZW=" in source
    assert "/action_sources/trajectory_test/joint_trajectory_goal" not in source
    assert "GoalStatusArray" not in source
    assert "scripts/send_mpd_trajectory.py" in cmake


def test_move_to_start_routes_smooth_official_franka_pose_to_jtc() -> None:
    source = _read("scripts/move_to_start.py")
    launch = _read("launch/move_to_start_demo.launch.py")
    assert "-math.pi / 4.0" in source
    assert "-3.0 * math.pi / 4.0" in source
    assert "math.pi / 2.0" in source
    assert "math.pi / 4.0" in source
    assert "/action_sources/trajectory_test/arm/joint_trajectory" in source
    assert "ActionClient" in source
    assert "get_result_async" in source
    assert '"duration_s", 10.0' in source
    assert '"num_points", 200' in source
    assert "JointTrajectory" in source
    assert '"/joint_states"' in source
    assert "_quintic" in source
    assert 'executable="move_to_start.py"' in launch
    assert '"duration_s"' in launch
    assert 'default_value="4.0"' in launch
    assert 'LaunchConfiguration("duration_s")' in launch
    assert "controller_manager" not in launch


def test_move_to_start_quintic_has_stationary_endpoints() -> None:
    namespace = runpy.run_path(PACKAGE_ROOT / "scripts/move_to_start.py")
    quintic = namespace["_quintic"]
    assert quintic(0.0) == pytest.approx((0.0, 0.0, 0.0))
    assert quintic(1.0) == pytest.approx((1.0, 0.0, 0.0))
    assert [quintic(step / 20.0)[0] for step in range(21)] == sorted(
        quintic(step / 20.0)[0] for step in range(21)
    )
