# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""All-in-one FR3 joint-target GUI demo: impedance JSPC + EM + tkinter.

```text
joint_target_gui
  -> /action_sources/joint_gui/joint_target
  -> manipulation_execution_manager
  -> /franka_arm_jspc/joint_reference
  -> JointSpaceImpedanceController (Ruckig)
  -> fake or real FR3
```

For distributed use, launch ``joint_gui_rt_bringup.launch.py`` on the RT host
and ``joint_gui_operator.launch.py`` on the operator PC.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('franka_trajectory_jtc_test')

    executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_share, 'launch', 'joint_gui_rt_bringup.launch.py']
            )
        ),
        launch_arguments={
            'controllers_yaml': LaunchConfiguration('controllers_yaml'),
            'execution_manager_yaml': LaunchConfiguration(
                'execution_manager_yaml'
            ),
            'arm_controller': LaunchConfiguration('arm_controller'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_franka_robot_state_broadcaster': LaunchConfiguration(
                'load_franka_robot_state_broadcaster'
            ),
        }.items(),
    )

    gui = Node(
        package='franka_trajectory_jtc_test',
        executable='joint_target_gui',
        name='joint_target_gui',
        output='screen',
        parameters=[LaunchConfiguration('operator_yaml')],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'controllers_yaml',
                default_value=PathJoinSubstitution(
                    [pkg_share, 'config', 'controllers_jspc.yaml']
                ),
                description='Impedance JSPC controller parameters.',
            ),
            DeclareLaunchArgument(
                'execution_manager_yaml',
                default_value=PathJoinSubstitution(
                    [pkg_share, 'config', 'execution_joint_gui_jspc.yaml']
                ),
                description='EM joint_gui joint_target → JSPC.',
            ),
            DeclareLaunchArgument(
                'arm_controller',
                default_value='franka_arm_jspc',
                description='Spawn franka_arm_jspc.',
            ),
            DeclareLaunchArgument(
                'operator_yaml',
                default_value=PathJoinSubstitution(
                    [pkg_share, 'config', 'joint_gui_operator_jspc.yaml']
                ),
                description='Operator GUI parameters (joint_target → JSPC).',
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
                description='Load vendor robot-state broadcaster on real hardware.',
            ),
            executor,
            gui,
        ]
    )
