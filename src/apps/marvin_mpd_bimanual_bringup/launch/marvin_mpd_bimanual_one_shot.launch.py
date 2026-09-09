"""Safe-default Phase-3 Marvin MPD one-shot demo."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("plan_only", default_value="true"),
            DeclareLaunchArgument("device", default_value="cuda:0"),
            DeclareLaunchArgument(
                "output_root", default_value="/tmp/mpd-marvin-bimanual-one-shot"
            ),
            Node(
                package="mpd_bimanual_planner_adapter",
                executable="marvin_bimanual_one_shot",
                name="marvin_mpd_bimanual_one_shot",
                output="screen",
                parameters=[
                    {
                        "plan_only": LaunchConfiguration("plan_only"),
                        "device": LaunchConfiguration("device"),
                        "output_root": LaunchConfiguration("output_root"),
                        "trajectory_action": "/bimanual_arm_jtc/follow_joint_trajectory",
                    }
                ],
            ),
        ]
    )
