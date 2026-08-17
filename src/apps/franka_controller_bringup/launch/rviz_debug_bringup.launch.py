# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""All-in-one Franka controller and RViz-marker debug bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose the pure controller service with the Franka marker profile."""
    bringup_share = FindPackageShare('franka_controller_bringup')

    controller_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [bringup_share, 'launch', 'controller_bringup.launch.py']
            )
        ),
        launch_arguments={
            'controllers_yaml': LaunchConfiguration('controllers_yaml'),
            'execution_manager_yaml': LaunchConfiguration(
                'execution_manager_yaml'
            ),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_franka_robot_state_broadcaster': LaunchConfiguration(
                'load_franka_robot_state_broadcaster'
            ),
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
            'profile': 'franka',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_markers')),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Launch RViz with the Franka profile.',
            ),
            DeclareLaunchArgument(
                'use_markers',
                default_value='true',
                description='Launch the Franka interactive-marker source.',
            ),
            DeclareLaunchArgument(
                'controllers_yaml',
                default_value=PathJoinSubstitution(
                    [bringup_share, 'config', 'controllers.yaml']
                ),
                description='controller_manager and TSJI parameter file.',
            ),
            DeclareLaunchArgument(
                'execution_manager_yaml',
                default_value=PathJoinSubstitution(
                    [bringup_share, 'config', 'execution_manager.yaml']
                ),
                description='Execution-manager route profile.',
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='true',
                description=(
                    'Use fake hardware. Set false only for a present, '
                    'powered, and safed FR3.'
                ),
            ),
            DeclareLaunchArgument(
                'robot_ip',
                default_value='192.168.2.101',
                description='FR3 hostname or IP; ignored by fake hardware.',
            ),
            DeclareLaunchArgument(
                'load_franka_robot_state_broadcaster',
                default_value='true',
                description=(
                    'Load the vendor state broadcaster on real hardware.'
                ),
            ),
            controller_bringup,
            marker_ui,
        ]
    )
