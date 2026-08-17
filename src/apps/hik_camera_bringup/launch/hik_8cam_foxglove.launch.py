"""8× Hikrobot + Foxglove Bridge (ws://localhost:8765)."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('hik_camera_bringup')
    foxglove_share = FindPackageShare('foxglove_bridge')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    bringup_share, 'launch', 'hik_8cam.launch.py',
                ]),
            ]),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    foxglove_share, 'launch', 'foxglove_bridge_launch.xml',
                ]),
            ]),
        ),
    ])
