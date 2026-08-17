# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""RT-side Franka trajectory executor: FR3 + effort JTC + EM.

Does not start a trajectory source. On the operator PC, run
``ros2 run franka_trajectory_jtc_test send_smooth_trajectory.py`` (or the
share/examples script) whenever you want to send one full JointTrajectory.
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
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    load_franka_robot_state_broadcaster = LaunchConfiguration(
        'load_franka_robot_state_broadcaster'
    ).perform(context)
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)
    execution_manager_yaml = LaunchConfiguration(
        'execution_manager_yaml'
    ).perform(context)
    arm_controller = LaunchConfiguration('arm_controller').perform(context)

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

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[arm_controller],
        output='screen',
    )

    execution_manager = Node(
        package='manipulation_execution_manager',
        executable='execution_manager',
        name='execution_manager',
        output='screen',
        parameters=[execution_manager_yaml],
    )

    return [franka_bringup, arm_spawner, execution_manager]


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('franka_trajectory_jtc_test')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'controllers_yaml',
                default_value=PathJoinSubstitution(
                    [pkg_share, 'config', 'controllers.yaml']
                ),
                description='controller_manager YAML (JTC or JSPC impedance).',
            ),
            DeclareLaunchArgument(
                'execution_manager_yaml',
                default_value=PathJoinSubstitution(
                    [pkg_share, 'config', 'execution_manager.yaml']
                ),
                description='EM route profile.',
            ),
            DeclareLaunchArgument(
                'arm_controller',
                default_value='fr3_arm_controller',
                description=(
                    'Controller to spawn: fr3_arm_controller (JTC) or '
                    'franka_arm_jspc (impedance).'
                ),
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
                    'hardware. Vendor bringup skips it on fake hardware.'
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
