"""Hardware-agnostic, config-driven launch composition over episode_recorder.

No topic or robot assumption is baked in. Every collection setup is selected
through config:

- `profile`: name of a bundled stream-contract YAML under this package's
  `config/` directory (e.g. `marvin_teleop_8cam_streams`). Use this to switch
  between different data-collection apps without writing a new package.
- `stream_config_uri`: absolute path to any stream-contract YAML, bundled or
  not. Takes precedence over `profile` when set.
- All other site values (root_dir, experiment_name, task, ...) are plain
  launch arguments with the same generic defaults as episode_recorder's own
  config/default_config.yaml.

Activation is driven by activate_lifecycle_node.py instead of the stock
OnProcessStart / ChangeState event-handler path, which races and can report
false TRANSITION_* failures before the node's services are ready.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare

_NODE_NAME = 'episode_recorder'


def _shutdown_on_activation_failure(event, _context):
    if event.returncode == 0:
        return []
    return [
        EmitEvent(
            event=Shutdown(
                reason=f'episode_recorder lifecycle activation failed '
                f'(exit code {event.returncode})'
            )
        )
    ]


def _resolve_stream_config_uri(context):
    explicit_uri = LaunchConfiguration('stream_config_uri').perform(context)
    if explicit_uri:
        return explicit_uri
    profile = LaunchConfiguration('profile').perform(context)
    if not profile:
        return ''
    return os.path.join(
        get_package_share_directory('episode_recorder_app'),
        'config',
        f'{profile}.yaml',
    )


def _launch_recorder(context, *args, **kwargs):
    activate_script = os.path.join(
        get_package_prefix('episode_recorder_app'),
        'lib',
        'episode_recorder_app',
        'activate_lifecycle_node.py',
    )

    recorder = LifecycleNode(
        package='episode_recorder',
        executable='episode_recorder_node',
        name=_NODE_NAME,
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'stream_config_uri': _resolve_stream_config_uri(context),
                'root_dir': LaunchConfiguration('root_dir'),
                'experiment_name': LaunchConfiguration('experiment_name'),
                'task': LaunchConfiguration('task'),
                'session_id': LaunchConfiguration('session_id'),
                'max_episode_duration': LaunchConfiguration('max_episode_duration'),
                'min_free_space_bytes': LaunchConfiguration('min_free_space_bytes'),
                'disk_reserve_bytes': LaunchConfiguration('disk_reserve_bytes'),
                'expected_max_bitrate_bytes_per_s': LaunchConfiguration(
                    'expected_max_bitrate_bytes_per_s'),
                'queue_capacity_messages': LaunchConfiguration(
                    'queue_capacity_messages'),
                'queue_capacity_bytes': LaunchConfiguration('queue_capacity_bytes'),
            },
        ],
    )

    lifecycle_autostart = ExecuteProcess(
        cmd=['python3', activate_script, f'/{_NODE_NAME}'],
        output='screen',
    )
    activation_failure_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_autostart,
            on_exit=_shutdown_on_activation_failure,
        )
    )

    return [recorder, lifecycle_autostart, activation_failure_handler]


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('episode_recorder'),
        'config',
        'default_config.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='',
            description=(
                'Name of a bundled stream-contract YAML under this package\'s '
                'config/ directory (without .yaml), e.g. marvin_teleop_8cam_streams. '
                'Ignored when stream_config_uri is set.'
            ),
        ),
        DeclareLaunchArgument(
            'stream_config_uri',
            default_value='',
            description=(
                'Absolute path to a stream-contract YAML. Takes precedence over '
                'profile. Empty and no profile falls back to the legacy topics '
                'parameter.'
            ),
        ),
        DeclareLaunchArgument(
            'root_dir',
            default_value='data/episodes',
            description='Episode storage root directory',
        ),
        DeclareLaunchArgument(
            'experiment_name',
            default_value='default',
            description='Experiment subdirectory under root_dir',
        ),
        DeclareLaunchArgument(
            'task',
            default_value='unspecified_task',
            description='Task string written into the episode manifest',
        ),
        DeclareLaunchArgument(
            'session_id',
            default_value='',
            description='Optional session id (empty lets the recorder assign one)',
        ),
        DeclareLaunchArgument(
            'max_episode_duration',
            default_value='0.0',
            description='Max episode duration in seconds (0 = unlimited)',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='episode_recorder node parameter YAML',
        ),
        DeclareLaunchArgument(
            'min_free_space_bytes',
            default_value='1073741824',
            description='Refuse to arm if free disk is below this many bytes',
        ),
        DeclareLaunchArgument(
            'disk_reserve_bytes',
            default_value='1073741824',
            description='Extra disk reserve beyond planned episode footprint',
        ),
        DeclareLaunchArgument(
            'expected_max_bitrate_bytes_per_s',
            default_value='0',
            description='Planned bitrate for disk preflight (0 disables)',
        ),
        DeclareLaunchArgument(
            'queue_capacity_messages',
            default_value='4096',
            description='Bounded capture queue message capacity',
        ),
        DeclareLaunchArgument(
            'queue_capacity_bytes',
            default_value='1073741824',
            description='Bounded capture queue byte capacity',
        ),
        OpaqueFunction(function=_launch_recorder),
    ])
