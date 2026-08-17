# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""FR3 joint-GUI operator UI (non-RT / local PC).

```text
joint_target_gui
  -> /action_sources/joint_gui/joint_trajectory_goal  (DDS to RT EM)
```

Pair with ``joint_gui_rt_bringup.launch.py`` on the RT / robot host. Same
``ROS_DOMAIN_ID`` (and CycloneDDS peers if needed) on both machines. The GUI
reads ``/franka/joint_states`` published by the robot host.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('franka_trajectory_jtc_test')

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
                'operator_yaml',
                default_value=PathJoinSubstitution(
                    [pkg_share, 'config', 'joint_gui_operator.yaml']
                ),
                description='Operator GUI / auto-send parameters.',
            ),
            gui,
        ]
    )
