# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Launch the execution manager from an installed parameter profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    namespace = LaunchConfiguration("namespace")
    status_rate_hz = LaunchConfiguration("status_rate_hz")

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("manipulation_execution_manager"),
            "config",
            "execution_manager.yaml",
        ]
    )

    node = Node(
        package="manipulation_execution_manager",
        executable="execution_manager",
        name="execution_manager",
        namespace=namespace,
        output="screen",
        parameters=[
            config_file,
            {
                "status_rate_hz": ParameterValue(
                    status_rate_hz, value_type=float
                )
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Execution-manager ROS parameter YAML.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional ROS namespace for the node.",
            ),
            DeclareLaunchArgument(
                "status_rate_hz",
                default_value="2.0",
                description="Status publication rate in Hz.",
            ),
            node,
        ]
    )
