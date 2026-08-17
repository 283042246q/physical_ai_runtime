# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Config-driven RViz interactive-marker pose source.

This local-PC app publishes PoseStamped targets only. Pair it with the
matching controller bringup on the robot/CPU host.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import yaml


_REQUIRED_MARKER_KEYS = {
    'name',
    'base_frame',
    'target_frame',
    'pose_topic',
    'server_namespace',
    'marker_name',
    'marker_description',
}


def _load_profile(context: LaunchContext) -> list[Node]:
    profile = context.perform_substitution(LaunchConfiguration('profile'))
    config_file = context.perform_substitution(
        LaunchConfiguration('config_file')
    )

    if config_file:
        config_path = Path(config_file)
    else:
        if not profile.replace('_', '').isalnum():
            raise RuntimeError(f'Invalid marker profile name: {profile!r}')
        config_path = (
            Path(get_package_share_directory('rviz_marker_teleop'))
            / 'config'
            / f'{profile}.yaml'
        )

    if not config_path.is_file():
        raise RuntimeError(f'Marker profile does not exist: {config_path}')

    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if not isinstance(config, dict):
        raise RuntimeError(f'Marker profile must be a mapping: {config_path}')

    markers = config.get('markers')
    if not isinstance(markers, list) or not markers:
        raise RuntimeError(
            f'Marker profile needs a non-empty markers list: {config_path}'
        )

    nodes = []
    marker_names = set()
    for marker in markers:
        if not isinstance(marker, dict):
            raise RuntimeError(
                f'Marker entries must be mappings: {config_path}'
            )
        missing = _REQUIRED_MARKER_KEYS - marker.keys()
        if missing:
            raise RuntimeError(
                f'Marker in {config_path} is missing: {sorted(missing)}'
            )
        if marker['name'] in marker_names:
            raise RuntimeError(
                f"Duplicate marker name {marker['name']!r}: {config_path}"
            )
        marker_names.add(marker['name'])
        nodes.append(
            Node(
                package='rviz_interactive_marker_pose_source',
                executable='target_marker_node.py',
                name=f"target_marker_{marker['name']}",
                output='screen',
                parameters=[
                    {
                        'base_frame': marker['base_frame'],
                        'target_frame': marker['target_frame'],
                        'pose_topic': marker['pose_topic'],
                        'publish_frequency': float(
                            marker.get('publish_frequency', 50.0)
                        ),
                        'server_namespace': marker['server_namespace'],
                        'marker_name': marker['marker_name'],
                        'marker_description': marker['marker_description'],
                        'publish_before_first_feedback': bool(
                            marker.get(
                                'publish_before_first_feedback',
                                False,
                            )
                        ),
                        'required_joint_state_topic': marker.get(
                            'required_joint_state_topic',
                            '',
                        ),
                    }
                ],
            )
        )

    rviz = config.get('rviz')
    if not isinstance(rviz, dict) or not {'package', 'config'} <= rviz.keys():
        raise RuntimeError(
            f'Marker profile needs rviz.package and rviz.config: {config_path}'
        )
    rviz_config = (
        Path(get_package_share_directory(rviz['package'])) / rviz['config']
    )
    if not rviz_config.is_file():
        raise RuntimeError(f'RViz config does not exist: {rviz_config}')
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', str(rviz_config)],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        )
    )
    return nodes


def generate_launch_description() -> LaunchDescription:
    """Declare profile selection and create its marker/RViz nodes."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'profile',
                default_value='marvin',
                description='Bundled marker profile: marvin or franka.',
            ),
            DeclareLaunchArgument(
                'config_file',
                default_value='',
                description=(
                    'Optional absolute profile YAML path. When set, it '
                    'overrides profile.'
                ),
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Launch RViz2 with the profile visualization.',
            ),
            OpaqueFunction(function=_load_profile),
        ]
    )
