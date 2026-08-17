"""Config-driven Marvin motion-planning bringup."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from marvin_motion_planning_bringup.planning_config import load_planning_config
import xacro


def _spawner(controller_name: str) -> Node:
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name, "--controller-manager", "/controller_manager"],
        output="screen",
    )


def _create_nodes(context: LaunchContext) -> list:
    package_share = get_package_share_directory("marvin_motion_planning_bringup")
    description_share = get_package_share_directory("marvin_description")
    planning_path = context.perform_substitution(LaunchConfiguration("planning_config"))
    plans = load_planning_config(planning_path)

    use_fake = context.perform_substitution(LaunchConfiguration("use_fake_hardware"))
    robot_description = xacro.process_file(
        str(Path(description_share) / "urdf" / "marvin.urdf.xacro"),
        mappings={
            "ros2_control": "true",
            "use_fake_hardware": use_fake,
            "hardware_plugin": context.perform_substitution(
                LaunchConfiguration("hardware_plugin")
            ),
            "robot_ip": context.perform_substitution(LaunchConfiguration("robot_ip")),
            "mounts_file": context.perform_substitution(
                LaunchConfiguration("mounts_file")
            ),
        },
    ).toprettyxml(indent="  ")

    controllers = str(Path(package_share) / "config" / "controllers.yaml")
    execution = str(Path(package_share) / "config" / "execution.yaml")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{"robot_description": robot_description}, controllers],
    )
    jsb = _spawner("joint_state_broadcaster")
    controller_spawners = [
        _spawner(
            f"{plan.side}_arm_jtc"
            if plan.mode == "global_trajectory"
            else f"{plan.side}_arm_jspc"
        )
        for plan in plans
    ]
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=jsb, on_exit=controller_spawners)
    )

    actions: list = [robot_state_publisher, control_node, jsb, delayed_controllers]
    trajectory_sides: list[str] = []
    for plan in plans:
        actions.append(
            Node(
                package="manipulation_execution_manager",
                executable="execution_manager",
                name=f"em_{plan.side}",
                output="screen",
                parameters=[execution],
            )
        )
        remappings = []
        if plan.mode == "global_trajectory":
            remappings.append(("/pyroki/execute", "/motion_planning/execute"))
            trajectory_sides.append(plan.side)
        actions.append(
            Node(
                package=plan.package,
                executable=plan.executable,
                name=f"motion_planner_{plan.side}",
                output="screen",
                parameters=[plan.parameters],
                remappings=remappings,
            )
        )
        if plan.mode != "global_trajectory":
            suffix = "L" if plan.side == "left" else "R"
            actions.append(
                Node(
                    package="rviz_interactive_marker_pose_source",
                    executable="target_marker_node.py",
                    name=f"{plan.side}_target_marker",
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("use_rviz")),
                    parameters=[
                        {
                            "pose_topic": plan.parameters["pose_topic"],
                            "publish_frequency": 30.0,
                            "base_frame": "world",
                            "target_frame": f"flange_{suffix}",
                            "server_namespace": f"motion_planning_{plan.side}_target",
                            "marker_name": f"{plan.side}_target",
                            "marker_description": f"{plan.side.capitalize()} target",
                            "publish_before_first_feedback": (
                                plan.mode != "online_mpc"
                            ),
                        }
                    ],
                )
            )

    if trajectory_sides:
        target_topics = {
            plan.side: plan.parameters["pose_topic"]
            for plan in plans
            if plan.side in trajectory_sides
        }
        actions.append(
            Node(
                package="marvin_motion_planning_bringup",
                executable="motion_planning_target_markers",
                name="motion_planning_target_markers",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                parameters=[
                    {
                        "enabled_arms": ",".join(trajectory_sides),
                        "left_topic": target_topics.get("left", "/teleop/pose_left"),
                        "right_topic": target_topics.get("right", "/teleop/pose_right"),
                    }
                ],
            )
        )
    rviz_file = (
        "marvin_global_trajectory.rviz"
        if trajectory_sides
        else "marvin_streaming_planner.rviz"
    )
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "--display-config",
                str(Path(package_share) / "rviz" / rviz_file),
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("marvin_motion_planning_bringup")
    description_share = get_package_share_directory("marvin_description")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "planning_config",
                default_value=str(Path(package_share) / "config" / "planning.yaml"),
                description="Per-arm mode, backend, target, and backend parameters.",
            ),
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument(
                "hardware_plugin",
                default_value="marvin_hardware_interface/MarvinBimanualArmHardware",
            ),
            DeclareLaunchArgument("robot_ip", default_value="10.19.0.191"),
            DeclareLaunchArgument(
                "mounts_file",
                default_value=str(Path(description_share) / "config" / "arm_mounts.yaml"),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            OpaqueFunction(function=_create_nodes),
        ]
    )
