# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Run the smooth-trajectory JTC demo through the server-owned EM."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="franka_motion_demos",
                executable="smooth_trajectory.py",
                name="smooth_trajectory",
                output="screen",
            )
        ]
    )
