# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Move the FR3 to the start configuration through EM and JTC."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "duration_s",
                default_value="4.0",
                description="Duration of the smooth move-to-start trajectory",
            ),
            Node(
                package="franka_motion_demos",
                executable="move_to_start.py",
                name="move_to_start",
                output="screen",
                parameters=[{"duration_s": LaunchConfiguration("duration_s")}],
            )
        ]
    )
