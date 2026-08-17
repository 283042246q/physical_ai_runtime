"""Launch the separate Phase-4 dynamic MPD adapter; worker is external."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("mpd_dynamic_planner_adapter"), "config", "replan_dynamic.yaml"]
    )
    arguments = [
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("plan_only", default_value="true"),
        DeclareLaunchArgument("plan_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("target_pose_xyzw", default_value=""),
        DeclareLaunchArgument("joint_state_topic", default_value="/franka/joint_states"),
        DeclareLaunchArgument(
            "world_observation_topic", default_value="/mpd/dynamic_world_observations"
        ),
        DeclareLaunchArgument(
            "jtc_action_name", default_value="/franka_arm_jtc/follow_joint_trajectory"
        ),
    ]
    replanner = Node(
        package="mpd_dynamic_planner_adapter",
        executable="dynamic_replan_node",
        name="mpd_dynamic_replanner",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {
                "plan_only": LaunchConfiguration("plan_only"),
                "plan_rate_hz": LaunchConfiguration("plan_rate_hz"),
                "target_pose_xyzw": ParameterValue(
                    LaunchConfiguration("target_pose_xyzw"), value_type=str
                ),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "world_observation_topic": LaunchConfiguration("world_observation_topic"),
                "jtc_action_name": LaunchConfiguration("jtc_action_name"),
            },
        ],
    )
    safe_stop = Node(
        package="mpd_planner_adapter",
        executable="jtc_safe_stop",
        name="mpd_dynamic_jtc_safe_stop",
        output="screen",
        parameters=[{"jtc_action_name": LaunchConfiguration("jtc_action_name")}],
    )
    return LaunchDescription([*arguments, replanner, safe_stop])
