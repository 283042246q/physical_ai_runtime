"""Phase-4 MPD replanning with the production Franka server on fake hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("franka_manipulation_controller_bringup"),
                    "launch",
                    "controller_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": "true",
            "activate_trajectory_controller": "true",
        }.items(),
    )
    dynamic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mpd_dynamic_planner_adapter"),
                    "launch",
                    "replan_dynamic.launch.py",
                ]
            )
        ),
        launch_arguments={
            "plan_only": LaunchConfiguration("plan_only"),
            "plan_rate_hz": LaunchConfiguration("plan_rate_hz"),
            "target_pose_xyzw": LaunchConfiguration("target_pose_xyzw"),
            "scene_id": LaunchConfiguration("scene_id"),
            "socket_path": LaunchConfiguration("socket_path"),
            "replay_record_dir": LaunchConfiguration("replay_record_dir"),
            "replay_env_name": LaunchConfiguration("replay_env_name"),
            "replay_static_scene_json": LaunchConfiguration(
                "replay_static_scene_json"
            ),
        }.items(),
    )
    demo = Node(
        package="mpd_dynamic_planner_adapter",
        executable="dynamic_world_demo",
        name="mpd_dynamic_world_demo",
        output="screen",
        parameters=[{"scenario": LaunchConfiguration("world_scenario")}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="192.168.2.101"),
            DeclareLaunchArgument("plan_only", default_value="true"),
            DeclareLaunchArgument("plan_rate_hz", default_value="0.5"),
            DeclareLaunchArgument("world_scenario", default_value="safe_far"),
            DeclareLaunchArgument("scene_id", default_value="EnvWarehouseExtraObjectsV00"),
            DeclareLaunchArgument("socket_path", default_value="/tmp/mpd-dynamic-runtime.sock"),
            DeclareLaunchArgument("replay_record_dir", default_value=""),
            DeclareLaunchArgument("replay_env_name", default_value=""),
            DeclareLaunchArgument("replay_static_scene_json", default_value=""),
            DeclareLaunchArgument(
                "target_pose_xyzw",
                default_value="0.4322543,0.1637504,0.6717085,0.4711762,0.0645563,-0.0740393,0.8765521",
            ),
            server,
            TimerAction(period=5.0, actions=[demo, dynamic_launch]),
        ]
    )
