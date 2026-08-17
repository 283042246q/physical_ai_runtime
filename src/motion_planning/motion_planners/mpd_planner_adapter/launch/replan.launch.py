"""Launch the Phase-2 resident MPD ROS adapter (planner worker is external)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("mpd_planner_adapter"), "config", "replan.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("plan_only", default_value="true"),
            DeclareLaunchArgument("plan_rate_hz", default_value="1.0"),
            DeclareLaunchArgument(
                "joint_state_topic", default_value="/franka/joint_states"
            ),
            DeclareLaunchArgument(
                "jtc_action_name",
                default_value="/franka_arm_jtc/follow_joint_trajectory",
            ),
            Node(
                package="mpd_planner_adapter",
                executable="replan_node",
                name="mpd_replanner",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "plan_only": LaunchConfiguration("plan_only"),
                        "plan_rate_hz": LaunchConfiguration("plan_rate_hz"),
                        "joint_state_topic": LaunchConfiguration(
                            "joint_state_topic"
                        ),
                        "jtc_action_name": LaunchConfiguration(
                            "jtc_action_name"
                        ),
                    },
                ],
            ),
            Node(
                package="mpd_planner_adapter",
                executable="jtc_safe_stop",
                name="mpd_jtc_safe_stop",
                output="screen",
                parameters=[
                    {
                        "jtc_action_name": LaunchConfiguration(
                            "jtc_action_name"
                        )
                    }
                ],
            ),
        ]
    )
