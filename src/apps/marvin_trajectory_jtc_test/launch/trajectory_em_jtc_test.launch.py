# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Safe fake-hardware trajectory -> EM -> JTC end-to-end probe."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
import xacro


def _nodes(context: LaunchContext):
    del context
    app_share = get_package_share_directory("marvin_trajectory_jtc_test")
    marvin_share = get_package_share_directory("marvin_description")
    robot_description = xacro.process_file(
        os.path.join(marvin_share, "urdf", "marvin.urdf.xacro"),
        mappings={"ros2_control": "true", "use_fake_hardware": "true"},
    ).toprettyxml(indent="  ")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(app_share, "config", "controllers.yaml"),
        ],
    )
    execution_manager = Node(
        package="manipulation_execution_manager",
        executable="execution_manager",
        name="trajectory_test_em",
        output="screen",
        parameters=[os.path.join(app_share, "config", "execution_manager.yaml")],
    )

    spawn_joint_states = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    probe = Node(
        package="marvin_trajectory_jtc_test",
        executable="trajectory_chain_probe.py",
        name="trajectory_chain_probe",
        output="screen",
    )

    return [
        robot_state_publisher,
        controller_manager,
        execution_manager,
        spawn_joint_states,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_joint_states, on_exit=[spawn_jtc])
        ),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jtc, on_exit=[probe])),
        RegisterEventHandler(
            OnProcessExit(
                target_action=probe,
                on_exit=[EmitEvent(event=Shutdown(reason="trajectory probe completed"))],
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=_nodes)])
