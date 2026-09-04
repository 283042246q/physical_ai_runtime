from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def _nodes(context):
    share = Path(get_package_share_directory("marvin_mpd_bimanual_bringup"))
    description = xacro.process_file(str(share / "urdf" / "marvin_pika_bimanual.urdf.xacro"), mappings={"use_fake_arm_hardware": "true", "use_fake_left_gripper_hardware": "true", "use_fake_right_gripper_hardware": "true"}).toprettyxml(indent="  ")
    controllers = str(share / "config" / "controllers_bimanual.yaml")
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": description}], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node", parameters=[{"robot_description": description}, controllers], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["bimanual_arm_jtc", "--controller-manager", "/controller_manager"], output="screen", condition=IfCondition(LaunchConfiguration("execute"))),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("execute", default_value="false"), OpaqueFunction(function=_nodes)])
