# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""All-in-one Marvin controller and RViz-marker debug bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose the pure controller service with the Marvin marker profile."""
    bringup_share = FindPackageShare('marvin_controller_bringup')

    controller_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [bringup_share, 'launch', 'controller_bringup.launch.py']
            )
        ),
        launch_arguments={
            'controllers_yaml': LaunchConfiguration('controllers_yaml'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'hardware_plugin': LaunchConfiguration('hardware_plugin'),
            'robot_ip': LaunchConfiguration('robot_ip'),
        }.items(),
    )

    marker_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('rviz_marker_teleop'),
                    'launch',
                    'rviz_marker_teleop.launch.py',
                ]
            )
        ),
        launch_arguments={
            'profile': 'marvin',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_markers')),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Launch RViz with the Marvin profile.',
            ),
            DeclareLaunchArgument(
                'use_markers',
                default_value='true',
                description='Launch both Marvin interactive-marker sources.',
            ),
            DeclareLaunchArgument(
                'controllers_yaml',
                default_value=PathJoinSubstitution(
                    [bringup_share, 'config', 'controllers.yaml']
                ),
                description='controller_manager and both TSKPC parameters.',
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='true',
                description=(
                    'Use fake hardware. Set false only for a present, '
                    'powered, and safed Marvin.'
                ),
            ),
            DeclareLaunchArgument(
                'hardware_plugin',
                default_value=(
                    'marvin_hardware_interface/MarvinBimanualArmHardware'
                ),
                description='Real Marvin ros2_control hardware plugin.',
            ),
            DeclareLaunchArgument(
                'robot_ip',
                default_value='10.19.0.191',
                description='Marvin controller IP; ignored by fake hardware.',
            ),
            controller_bringup,
            marker_ui,
        ]
    )
