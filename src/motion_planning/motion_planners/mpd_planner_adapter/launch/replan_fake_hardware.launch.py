"""Resident MPD replanning with the production Franka server on mock hardware."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    replan_config = PathJoinSubstitution(
        [FindPackageShare("mpd_planner_adapter"), "config", "replan.yaml"]
    )
    franka_server = IncludeLaunchDescription(
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
    replanner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="mpd_planner_adapter",
                executable="replan_node",
                name="mpd_replanner",
                output="screen",
                parameters=[
                    replan_config,
                    {
                        "plan_only": LaunchConfiguration("plan_only"),
                        "plan_rate_hz": LaunchConfiguration("plan_rate_hz"),
                        "joint_state_topic": LaunchConfiguration(
                            "joint_state_topic"
                        ),
                        "jtc_action_name": LaunchConfiguration(
                            "jtc_action_name"
                        ),
                        "target_pose_xyzw": [
                            0.4322543,
                            0.1637504,
                            0.6717085,
                            0.4711762,
                            0.0645563,
                            -0.0740393,
                            0.8765521,
                        ],
                    },
                ],
            )
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="192.168.2.101"),
            DeclareLaunchArgument("plan_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("plan_only", default_value="true"),
            DeclareLaunchArgument(
                "joint_state_topic", default_value="/franka/joint_states"
            ),
            DeclareLaunchArgument(
                "jtc_action_name",
                default_value="/franka_arm_jtc/follow_joint_trajectory",
            ),
            franka_server,
            replanner,
            TimerAction(
                period=5.0,
                actions=[
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
                    )
                ],
            ),
        ]
    )
