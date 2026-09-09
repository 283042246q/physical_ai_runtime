from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("runtime_mode", default_value="snapshot_no_time"),
        DeclareLaunchArgument("worker_socket", default_value="/tmp/mpd_marvin_bimanual_static.sock"),
        Node(
            package="mpd_bimanual_planner_adapter",
            executable="bimanual_planner_node",
            name="mpd_bimanual_planner_adapter",
            namespace="marvin/mpd_bimanual",
            output="screen",
            parameters=[{
                "execute": LaunchConfiguration("execute"),
                "runtime_mode": LaunchConfiguration("runtime_mode"),
                "worker_socket": LaunchConfiguration("worker_socket"),
            }],
        ),
    ])
