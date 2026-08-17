# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""RViz target -> PyRoki global IK -> EM -> Franka JSIC."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_share = Path(
        get_package_share_directory("franka_description")
    )
    pose_topic = "/motion_demos/global_ik/franka_target"

    return LaunchDescription(
        [
            Node(
                package="pyroki_planner_adapter",
                executable="pyroki_global_setpoint_planner",
                name="motion_planner",
                output="screen",
                parameters=[
                    {
                        "target_link_name": "fr3_link8",
                        "robot_description_node": "robot_state_publisher",
                        "source_name": "motion_planner",
                        "source_namespace": "/action_sources",
                        "command_sink_mode": "em",
                        "pose_topic": pose_topic,
                        "output_joint_names": ",".join(
                            f"fr3_joint{index}" for index in range(1, 8)
                        ),
                        "load_meshes": False,
                        "plan_rate_hz": 50.0,
                        "max_state_age_s": 0.1,
                        "pose_stale_timeout_s": 0.5,
                        "method": "jparse",
                        "gamma": 0.3,
                        "position_gain": 15.0,
                        "orientation_gain": 3.0,
                        "nullspace_gain": 0.05,
                        "max_joint_velocity": 2.5,
                        "dls_damping": 0.05,
                        "max_iterations_per_tick": 4,
                        "max_step_rad": 0.05,
                        "position_tolerance_m": 1.0e-3,
                        "orientation_tolerance_rad": 1.0e-2,
                    }
                ],
            ),
            Node(
                package="rviz_interactive_marker_pose_source",
                executable="target_marker_node.py",
                name="global_ik_target_marker",
                output="screen",
                parameters=[
                    {
                        "base_frame": "fr3_link0",
                        "target_frame": "fr3_link8",
                        "pose_topic": pose_topic,
                        "publish_frequency": 30.0,
                        "server_namespace": "franka_global_ik_target",
                        "marker_name": "franka_global_ik_target",
                        "marker_description": "Franka Global IK Target",
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
