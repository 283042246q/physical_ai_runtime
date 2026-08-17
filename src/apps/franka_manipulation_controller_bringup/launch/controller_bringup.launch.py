# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Physical AI Runtime controller bringup for one Franka FR3 arm.

The vendor package owns robot description and ros2_control hardware startup.
This app adds the runtime execution manager and TaskSpaceJointImpedance
composition (Diff-IK + joint impedance → effort).
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _continue_or_shutdown(event, next_actions, stage):
    if event.returncode == 0:
        return next_actions
    reason = f'{stage} failed with exit code {event.returncode}'
    return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]


def _resolve_cpu_affinity(context) -> str:
    """Prefer launch arg; else RT_CM_CPU_AFFINITY from the cpu RT profile."""
    explicit = LaunchConfiguration('cpu_affinity').perform(context).strip()
    if explicit in ('none', 'off', '-'):
        return ''
    if explicit:
        return explicit
    return os.environ.get('RT_CM_CPU_AFFINITY', '').strip()


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
    fake_controller_overrides_yaml = LaunchConfiguration(
        'fake_controller_overrides_yaml'
    ).perform(context)
    activate_trajectory_controller = LaunchConfiguration(
        'activate_trajectory_controller'
    ).perform(context).strip().lower() in ('1', 'true', 'yes', 'on')
    cpu_affinity = _resolve_cpu_affinity(context)

    franka_launch_arguments = {
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
        'cpu_affinity': cpu_affinity,
    }

    actions = []
    if cpu_affinity:
        actions.append(
            LogInfo(
                msg=(
                    f'Pinning ros2_control_node to CPUs {cpu_affinity} '
                    '(taskset; from cpu_affinity or RT_CM_CPU_AFFINITY).'
                )
            )
        )

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
        launch_arguments=franka_launch_arguments.items(),
    )

    inactive_controllers = [
        'franka_arm_tsjic',
        'franka_arm_jsic',
    ]
    if not activate_trajectory_controller:
        inactive_controllers.append('franka_arm_jtc')
    spawner_arguments = [
        *inactive_controllers,
        '--inactive',
        '--controller-manager',
        '/controller_manager',
    ]
    if use_fake_hardware == 'true':
        spawner_arguments.extend(
            ['--param-file', fake_controller_overrides_yaml]
        )
    inactive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=spawner_arguments,
        output='screen',
    )

    trajectory_controller_spawner = None
    if activate_trajectory_controller:
        trajectory_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'franka_arm_jtc',
                '--controller-manager',
                '/controller_manager',
            ],
            output='screen',
        )
        actions.append(
            LogInfo(
                msg=(
                    'Activating franka_arm_jtc for an external owned '
                    'FollowJointTrajectory client.'
                )
            )
        )

    execution_manager = Node(
        package='manipulation_execution_manager',
        executable='execution_manager',
        name='execution_manager',
        output='screen',
        parameters=[execution_manager_yaml],
    )

    actions.extend(
        [
            franka_bringup,
            inactive_controller_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=inactive_controller_spawner,
                    on_exit=lambda event, context: _continue_or_shutdown(
                        event,
                        (
                            [trajectory_controller_spawner]
                            if trajectory_controller_spawner is not None
                            else [execution_manager]
                        ),
                        'inactive controller startup',
                    ),
                )
            ),
        ]
    )
    if trajectory_controller_spawner is not None:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=trajectory_controller_spawner,
                    on_exit=lambda event, context: _continue_or_shutdown(
                        event,
                        [execution_manager],
                        'trajectory controller startup',
                    ),
                )
            )
        )
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=execution_manager,
                on_exit=lambda event, context: [
                    EmitEvent(
                        event=Shutdown(
                            reason=(
                                'execution manager exited with code '
                                f'{event.returncode}'
                            )
                        )
                    )
                ],
            )
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Compose the vendor FR3 bringup with the Physical AI control path."""
    bringup_share = FindPackageShare(
        'franka_manipulation_controller_bringup'
    )

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
                'fake_controller_overrides_yaml',
                default_value=PathJoinSubstitution(
                    [
                        bringup_share,
                        'config',
                        'controllers_fake_overrides.yaml',
                    ]
                ),
                description=(
                    'Controller overrides used only with fake hardware.'
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
                'activate_trajectory_controller',
                default_value='false',
                description=(
                    'Activate franka_arm_jtc at startup for an external '
                    'FollowJointTrajectory owner such as the MPD replanner. '
                    'The default keeps every route controller inactive.'
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
            DeclareLaunchArgument(
                'cpu_affinity',
                default_value='',
                description=(
                    'Comma-separated CPUs for ros2_control_node taskset. '
                    'Empty uses RT_CM_CPU_AFFINITY from the cpu RT profile '
                    '(see docs/CPU_HOST_SETUP.md). Pass an explicit list to '
                    'override.'
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
