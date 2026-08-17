# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Franka motion-planning low-level service (RT / robot host).

Starts only the control path that should stay close to the arm:

```text
execution_manager
  -> JointSpaceImpedanceController (Ruckig)
  -> franka_bringup / ros2_control (effort)
```

Run the operator UI (planner + marker + optional RViz) separately with
`operator_bringup.launch.py` on a non-RT machine when using distributed DDS.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    # Perform substitutions before including franka.launch.py so nested
    # LaunchConfiguration('use_fake_hardware') cannot collapse to the child's
    # default (false) and silently select real hardware.
    share = FindPackageShare('franka_motion_planning_bringup').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    load_franka_robot_state_broadcaster = LaunchConfiguration(
        'load_franka_robot_state_broadcaster'
    ).perform(context)
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)
    execution_manager_yaml = LaunchConfiguration(
        'execution_manager_yaml'
    ).perform(context)

    franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py']
            )
        ),
        launch_arguments={
            'robot_type': 'fr3',
            'arm_prefix': '',
            'namespace': '',
            'robot_ip': robot_ip,
            'load_gripper': 'false',
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': 'false',
            'joint_state_rate': '100',
            'load_franka_robot_state_broadcaster': (
                load_franka_robot_state_broadcaster
            ),
            'controllers_yaml': controllers_yaml,
        }.items(),
    )

    jspc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_arm_jspc'],
        output='screen',
    )
    execution_manager = Node(
        package='manipulation_execution_manager',
        executable='execution_manager',
        name='execution_manager',
        output='screen',
        parameters=[execution_manager_yaml],
    )

    return [franka, jspc, execution_manager]


def generate_launch_description() -> LaunchDescription:
    """Compose vendor FR3 bringup with EM + JSPC(Ruckig) only."""
    bringup_share = FindPackageShare('franka_motion_planning_bringup')

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
                'use_fake_hardware',
                default_value='true',
                description=(
                    'Use mock_components/GenericSystem. Set false only for a '
                    'present, powered, and safed FR3.'
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
                    'Load the vendor robot-state broadcaster on real '
                    'hardware. The vendor bringup skips it on fake hardware.'
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
