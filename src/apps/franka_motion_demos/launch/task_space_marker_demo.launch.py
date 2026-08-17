# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""RViz target pose -> EM -> Franka TSJIC."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_share = Path(
        get_package_share_directory("franka_description")
    )

    return LaunchDescription(
        [
            Node(
                package="rviz_interactive_marker_pose_source",
                executable="target_marker_node.py",
                name="franka_task_space_target_marker",
                output="screen",
                parameters=[
                    {
                        "base_frame": "fr3_link0",
                        "target_frame": "fr3_link8",
                        "pose_topic": (
                            "/action_sources/marker/arm/cartesian_pose"
                        ),
                        "publish_frequency": 50.0,
                        "server_namespace": "franka_task_space_target",
                        "marker_name": "franka_task_space_target",
                        "marker_description": "Franka Task-Space Target",
                        "publish_before_first_feedback": False,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "--display-config",
                    str(
                        description_share
                        / "rviz"
                        / "visualize_franka.rviz"
                    ),
                ],
            ),
        ]
    )
