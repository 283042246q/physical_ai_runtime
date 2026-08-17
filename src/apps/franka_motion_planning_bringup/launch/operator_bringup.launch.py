# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Franka motion-planning operator UI (non-RT / local PC).

Starts only the high-level planning sources:

```text
RViz interactive marker
  -> PyRoki global setpoint planner
  -> /action_sources/motion_planner/*  (DDS to the robot host EM)
```

Pair with `planning_bringup.launch.py` on the RT / robot host. Same
`ROS_DOMAIN_ID` (and CycloneDDS peers if needed) on both machines. The planner
reads `robot_description` from the remote `robot_state_publisher` and uses
`/franka/joint_states` + TF published by the robot host.
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
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    planner_yaml = LaunchConfiguration('planner_yaml').perform(context)
    marker_config = LaunchConfiguration('marker_config').perform(context)

    planner = Node(
        package='pyroki_planner_adapter',
        executable='pyroki_global_setpoint_planner',
        name='motion_planner',
        output='screen',
        parameters=[planner_yaml],
    )
    marker = IncludeLaunchDescription(
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
            'config_file': marker_config,
            'use_rviz': use_rviz,
        }.items(),
    )

    return [planner, marker]


def generate_launch_description() -> LaunchDescription:
    """Compose planner + Franka planning marker profile."""
    bringup_share = FindPackageShare('franka_motion_planning_bringup')

    return LaunchDescription(
        [
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
                'use_rviz',
                default_value='true',
                description='Launch RViz with the marker profile config.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
