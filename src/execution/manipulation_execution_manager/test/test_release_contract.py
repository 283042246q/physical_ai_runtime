from pathlib import Path
import xml.etree.ElementTree as ET

import manipulation_execution_manager
from manipulation_execution_manager.execution_manager_node import ExecutionManager
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_version_matches_package_xml():
    package_version = ET.parse(PACKAGE_ROOT / "package.xml").findtext("version")
    assert manipulation_execution_manager.__version__ == package_version


def test_public_release_files_exist():
    for relative_path in (
        "LICENSE",
        "README.md",
        "CHANGELOG.md",
        "package.xml",
        "setup.py",
        "setup.cfg",
        "resource/manipulation_execution_manager",
    ):
        assert (PACKAGE_ROOT / relative_path).is_file(), relative_path


def test_runtime_does_not_patch_python_path():
    node_source = (
        PACKAGE_ROOT
        / "manipulation_execution_manager"
        / "execution_manager_node.py"
    ).read_text(encoding="utf-8")
    assert "sys.path" not in node_source


def test_entry_point_exit_module_is_imported():
    node_source = (
        PACKAGE_ROOT
        / "manipulation_execution_manager"
        / "execution_manager_node.py"
    ).read_text(encoding="utf-8")
    assert "import sys as _sys" in node_source


def test_public_package_has_no_milestone_modules():
    package_dir = PACKAGE_ROOT / "manipulation_execution_manager"
    names = {path.name for path in package_dir.glob("*.py")}
    assert not any(name.startswith(("m1_", "m2_", "m6_")) for name in names)


def test_public_entry_points_are_minimal():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert "execution_manager =" in setup_source
    assert "safety_monitor =" not in setup_source
    assert "twist_stamped_relay =" not in setup_source
    assert "m1_" not in setup_source
    assert "m2_" not in setup_source
    assert "m6_" not in setup_source


def test_jtc_goal_logging_uses_rclpy_logger_signature():
    class FakeClient:
        def server_is_ready(self):
            return True

        def send_goal_async(self, goal):
            self.goal = goal

    class FakeLogger:
        def debug(self, message):
            self.message = message

    manager = object.__new__(ExecutionManager)
    manager._jtc_client = FakeClient()
    manager._jtc_action_name = "/test/follow_joint_trajectory"
    logger = FakeLogger()
    manager.get_logger = lambda: logger
    trajectory = JointTrajectory(
        joint_names=["joint_1"], points=[JointTrajectoryPoint(positions=[0.0])]
    )

    manager._send_jtc_goal(trajectory)

    assert manager._jtc_client.goal.trajectory == trajectory
    assert logger.message == "JTC goal sent: 1 points, 1 joints"
