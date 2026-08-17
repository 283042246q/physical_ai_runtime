# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""All-in-one Franka planning bringup (local fake or single-host debug).

Composes the low-level planning service with the operator UI, matching the
controller bringup pattern of `controller_bringup` + `rviz_debug_bringup`.

For distributed RT testing prefer the split launches instead:

- robot/RT host: `planning_bringup.launch.py`
- operator PC: `operator_bringup.launch.py`
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose planning_bringup with operator_bringup on one machine."""
    bringup_share = FindPackageShare('franka_motion_planning_bringup')

    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [bringup_share, 'launch', 'planning_bringup.launch.py']
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

    operator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [bringup_share, 'launch', 'operator_bringup.launch.py']
            )
        ),
        launch_arguments={
            'planner_yaml': LaunchConfiguration('planner_yaml'),
            'marker_config': LaunchConfiguration('marker_config'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'controllers_yaml',
                default_value=PathJoinSubstitution(
                    [bringup_share, 'config', 'controllers.yaml']
                ),
                description='controller_manager and JSPC(Ruckig) parameters.',
            ),
            DeclareLaunchArgument(
                'execution_manager_yaml',
                default_value=PathJoinSubstitution(
                    [bringup_share, 'config', 'execution.yaml']
                ),
                description='Execution-manager route profile.',
            ),
            DeclareLaunchArgument(
                'planner_yaml',
                default_value=PathJoinSubstitution(
                    [bringup_share, 'config', 'planner.yaml']
                ),
                description='PyRoki global-setpoint planner parameters.',
            ),
            DeclareLaunchArgument(
                'marker_config',
                default_value=PathJoinSubstitution(
                    [bringup_share, 'config', 'marker.yaml']
                ),
                description='RViz marker profile for the planning pose topic.',
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
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Launch RViz with the marker profile config.',
            ),
            planning,
            operator,
        ]
    )
