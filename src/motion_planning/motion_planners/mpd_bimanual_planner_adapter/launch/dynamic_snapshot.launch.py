"""Phase-5 latest-only Marvin snapshot replanning."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument(
                "worker_socket", default_value="/tmp/mpd_marvin_bimanual_dynamic.sock"
            ),
            Node(
                package="mpd_bimanual_planner_adapter",
                executable="marvin_bimanual_dynamic_planner",
                output="screen",
                parameters=[
                    {
                        "execute": LaunchConfiguration("execute"),
                        "worker_socket": LaunchConfiguration("worker_socket"),
                    }
                ],
            ),
        ]
    )
