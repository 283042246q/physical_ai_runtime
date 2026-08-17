# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Physical AI Runtime controller bringup for one Franka FR3 arm.

The vendor package owns robot description and ros2_control hardware startup.
This app adds the runtime execution manager and TaskSpaceJointImpedance
composition (Diff-IK + joint impedance → effort).
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
    # Perform substitutions to plain strings before including franka.launch.py.
    # Passing LaunchConfiguration('use_fake_hardware') into a child that also
    # Declares the same name can resolve to the child's default (false) and
    # silently select the real FrankaHardwareInterface.
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    load_franka_robot_state_broadcaster = LaunchConfiguration(
        'load_franka_robot_state_broadcaster'
    ).perform(context)
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)
    execution_manager_yaml = LaunchConfiguration(
        'execution_manager_yaml'
    ).perform(context)

    franka_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('franka_bringup'),
                    'launch',
                    'franka.launch.py',
                ]
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

    tsji_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['task_space_joint_impedance_controller'],
        output='screen',
    )

    execution_manager = Node(
        package='manipulation_execution_manager',
        executable='execution_manager',
        name='execution_manager',
        output='screen',
        parameters=[execution_manager_yaml],
    )

    return [franka_bringup, tsji_spawner, execution_manager]


def generate_launch_description() -> LaunchDescription:
    """Compose the vendor FR3 bringup with the Physical AI control path."""
    bringup_share = FindPackageShare('franka_controller_bringup')

    return LaunchDescription(
        [
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
                    'hardware. The vendor bringup skips it automatically on '
                    'fake hardware.'
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
