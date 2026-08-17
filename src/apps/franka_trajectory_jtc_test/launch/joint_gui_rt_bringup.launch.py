# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""FR3 joint-GUI low-level service (RT / robot host) via effort JTC.

```text
execution_manager
  -> /fr3_arm_controller/follow_joint_trajectory
  -> JointTrajectoryController (effort + PID)
  -> franka_bringup / ros2_control
```

For impedance JSPC A/B, override:

  controllers_yaml:=.../controllers_jspc.yaml
  execution_manager_yaml:=.../execution_joint_gui_jspc.yaml
  arm_controller:=franka_arm_jspc

Run the operator GUI separately with ``joint_gui_operator.launch.py``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('franka_trajectory_jtc_test')

    executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_share, 'launch', 'trajectory_executor.launch.py']
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'controllers_yaml',
                default_value=PathJoinSubstitution(
                    [pkg_share, 'config', 'controllers.yaml']
                ),
                description='Effort JTC controller parameters.',
            ),
            DeclareLaunchArgument(
                'execution_manager_yaml',
                default_value=PathJoinSubstitution(
                    [pkg_share, 'config', 'execution_joint_gui.yaml']
                ),
                description='EM profile: joint_gui trajectory_goal → JTC.',
            ),
            DeclareLaunchArgument(
                'arm_controller',
                default_value='fr3_arm_controller',
                description='Spawn effort JointTrajectoryController.',
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
        ]
    )
