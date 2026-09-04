from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("marvin_mpd_bimanual_bringup"), "launch", "marvin_mpd_bimanual_fake_hardware.launch.py"])), launch_arguments={"execute": "false"}.items())])
