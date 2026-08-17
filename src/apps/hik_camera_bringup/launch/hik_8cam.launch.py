"""8× Hikrobot GigE: strict PTP Scheduled Action (site bringup)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('hik_camera_bringup'), 'config', 'hik_8cam.yaml'])
    hik_camera_launch = PathJoinSubstitution([
        FindPackageShare('hik_camera'), 'launch', 'hik_camera.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Absolute path to hik_camera ROS parameter YAML'),
        DeclareLaunchArgument(
            'validate_only',
            default_value='false',
            description='Validate configuration and exit without opening hardware'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([hik_camera_launch]),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'validate_only': LaunchConfiguration('validate_only'),
            }.items(),
        ),
    ])
