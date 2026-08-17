#!/usr/bin/env python3
"""Live/replay Quest3 bimanual source with IsaacTeleop session and RViz."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    cloudxr_install_dir = LaunchConfiguration("cloudxr_install_dir")
    cloudxr_env_config = LaunchConfiguration("cloudxr_env_config")
    teleop_rate_hz = LaunchConfiguration("teleop_rate_hz")
    mcap_replay_path = LaunchConfiguration("mcap_replay_path")
    pose_source = LaunchConfiguration("pose_source")
    deadman_source = LaunchConfiguration("deadman_source")
    deadman_threshold = LaunchConfiguration("deadman_threshold")
    require_both_deadman = LaunchConfiguration("require_both_deadman")
    linear_scale = LaunchConfiguration("linear_scale")
    angular_scale = LaunchConfiguration("angular_scale")
    lowpass_alpha = LaunchConfiguration("lowpass_alpha")
    profile_config = LaunchConfiguration("profile_config")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_delay_s = LaunchConfiguration("rviz_delay_s")
    left_base_frame = LaunchConfiguration("left_base_frame")
    right_base_frame = LaunchConfiguration("right_base_frame")

    default_profile_config = PathJoinSubstitution(
        [FindPackageShare("isaacteleop_toolbox"), "configs", "quest3_bimanual_relative.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("isaacteleop_toolbox"), "rviz", "isaacteleop_controller_replay.rviz"]
    )
    default_cloudxr_dir = EnvironmentVariable("CLOUDXR_DIR")
    default_cloudxr_env = PathJoinSubstitution(
        [default_cloudxr_dir, "cloudxr-env-config.env"]
    )

    quest3_source = Node(
        package="isaacteleop_toolbox",
        executable="quest3_bimanual_target",
        name="quest3_bimanual_target",
        output="screen",
        parameters=[
            profile_config,
            {
                "cloudxr_install_dir": cloudxr_install_dir,
                "cloudxr_accept_eula": True,
                "cloudxr_env_config": cloudxr_env_config,
                "cloudxr_host_client": True,
                "rate_hz": teleop_rate_hz,
                "mcap_replay_path": mcap_replay_path,
                "pose_source": pose_source,
                "deadman_source": deadman_source,
                "deadman_threshold": deadman_threshold,
                "require_both_deadman": require_both_deadman,
                "linear_scale": linear_scale,
                "angular_scale": angular_scale,
                "lowpass_alpha": lowpass_alpha,
                # No real robot TF tree in source-only mode: default both arm
                # base frames to output_frame so the world->base_frame lookup
                # is a trivial (identity) same-frame TF resolution, and the
                # node actually publishes PoseStamped/TF instead of silently
                # dropping every reference (see _publish_step's
                # `left_output_topic and left_base_frame` gate).
                "left_base_frame": left_base_frame,
                "right_base_frame": right_base_frame,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["--display-config", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
        name="rviz2",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cloudxr_install_dir",
                default_value=default_cloudxr_dir,
                description=(
                    "Prepared CloudXR data directory. Defaults to the "
                    "workspace-owned CLOUDXR_DIR."
                ),
            ),
            DeclareLaunchArgument(
                "cloudxr_env_config",
                default_value=default_cloudxr_env,
                description="CloudXR environment configuration file.",
            ),
            DeclareLaunchArgument("teleop_rate_hz", default_value="30.0"),
            DeclareLaunchArgument(
                "mcap_replay_path",
                default_value="",
                description="Optional IsaacTeleop DeviceIO MCAP replay path. Empty uses live Quest3/OpenXR.",
            ),
            DeclareLaunchArgument("pose_source", default_value="aim"),
            DeclareLaunchArgument("deadman_source", default_value="squeeze"),
            DeclareLaunchArgument("deadman_threshold", default_value="0.5"),
            DeclareLaunchArgument("require_both_deadman", default_value="true"),
            DeclareLaunchArgument("linear_scale", default_value="1.0"),
            DeclareLaunchArgument("angular_scale", default_value="1.0"),
            DeclareLaunchArgument("lowpass_alpha", default_value="0.35"),
            DeclareLaunchArgument(
                "profile_config",
                default_value=default_profile_config,
                description="ROS parameter YAML for the teleop adapter profile.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value=EnvironmentVariable("TELEOP_WITH_RVIZ", default_value="true"),
            ),
            DeclareLaunchArgument("rviz_delay_s", default_value="4.0"),
            DeclareLaunchArgument(
                "left_base_frame",
                default_value="world",
                description="Frame_id for published left pose_target. "
                "Defaults to output_frame (world) since this source-only "
                "launch has no real robot TF tree.",
            ),
            DeclareLaunchArgument(
                "right_base_frame",
                default_value="world",
                description="Frame_id for published right pose_target. "
                "Defaults to output_frame (world) since this source-only "
                "launch has no real robot TF tree.",
            ),
            SetEnvironmentVariable(
                "ROS_LOCALHOST_ONLY",
                EnvironmentVariable("ROS_LOCALHOST_ONLY", default_value="1"),
            ),
            SetEnvironmentVariable(
                "TELEOP_WEB_CLIENT_STATIC_DIR",
                [cloudxr_install_dir, "/static-client"],
            ),
            quest3_source,
            TimerAction(period=rviz_delay_s, actions=[rviz]),
        ]
    )
