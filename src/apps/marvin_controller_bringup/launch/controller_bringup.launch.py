# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Marvin bimanual controller bringup for a robot/CPU host.

Starts robot_state_publisher, ros2_control, both execution managers, and both
TaskSpaceKinematicPositionController instances. Interactive markers are not
part of this launch; run rviz_marker_teleop on the operator PC.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def _controller_nodes(context: LaunchContext):
    marvin_share = get_package_share_directory("marvin_description")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    robot_description_xacro = os.path.join(
        marvin_share, "urdf", "marvin.urdf.xacro"
    )
    robot_description = xacro.process_file(
        robot_description_xacro,
        mappings={
            "ros2_control": "true",
            "use_fake_hardware": context.perform_substitution(
                LaunchConfiguration("use_fake_hardware")
            ),
            "hardware_plugin": context.perform_substitution(
                LaunchConfiguration("hardware_plugin")
            ),
            "robot_ip": context.perform_substitution(LaunchConfiguration("robot_ip")),
        },
    ).toprettyxml(indent="  ")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    left_tskpc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_task_space_kinematic_position_controller"],
        output="screen",
    )

    right_tskpc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_task_space_kinematic_position_controller"],
        output="screen",
    )

    return [
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[left_tskpc_spawner, right_tskpc_spawner],
            )
        ),
    ]


def _execution_manager_node(side: str) -> Node:
    bringup_share = get_package_share_directory("marvin_controller_bringup")
    return Node(
        package="manipulation_execution_manager",
        executable="execution_manager",
        name=f"em_{side}",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [bringup_share, "config", f"execution_manager_{side}.yaml"]
            )
        ],
    )


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("marvin_controller_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "controllers.yaml"]
                ),
                description="controller_manager + TSKPC parameter file (both arms).",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description=(
                    "true: mock_components/GenericSystem (default, safe). "
                    "false: real Marvin SDK bridge -- only with the robot "
                    "present, powered, and safed."
                ),
            ),
            DeclareLaunchArgument(
                "hardware_plugin",
                default_value="marvin_hardware_interface/MarvinBimanualArmHardware",
                description=(
                    "Real ros2_control hardware plugin "
                    "(used when use_fake_hardware:=false)."
                ),
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.19.0.191",
                description=(
                    "Marvin controller IP (used when use_fake_hardware:=false)."
                ),
            ),
            OpaqueFunction(function=_controller_nodes),
            _execution_manager_node("left"),
            _execution_manager_node("right"),
        ]
    )
