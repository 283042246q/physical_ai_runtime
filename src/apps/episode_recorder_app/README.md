# episode_recorder_app

Hardware-agnostic, config-driven launch composition over
[`episode_recorder`](https://github.com/Gabriel-Ning/episode_recorder)
(provided by the `ros-jazzy-episode-recorder` runtime package or a workspace
source build). No topic or robot assumption is baked into code — every
data-collection setup is selected through config.

## What this package adds over the bare `episode_recorder` package

- **Profiles**: bundled stream-contract YAMLs under `config/`, selected by
  name via the `profile` launch argument. Switching to a different rig or
  sensor set is a config change, not a new package.
- **Arbitrary configs**: `stream_config_uri` accepts any absolute path and
  takes precedence over `profile`, for one-off or external stream contracts.
- **A more rigorous lifecycle activation path**: `scripts/activate_lifecycle_node.py`
  drives the node from `unconfigured` to `active` via direct
  `GetState`/`ChangeState` service calls and checks each transition's `success`
  result instead of blindly retrying. If activation fails, the whole launch
  shuts down rather than leaving an unusable inactive process behind.

## Build

```bash
pixi run build --packages-up-to episode_recorder_app
source install/setup.bash
```

## Use a bundled profile

```bash
ros2 launch episode_recorder_app record_episode.launch.py \
  profile:=marvin_teleop_8cam_streams \
  root_dir:=$WORKSPACE_ROOT/data/episodes \
  experiment_name:=marvin_teleop \
  task:=marker_teleop
```

## Use an arbitrary stream config (no profile)

```bash
ros2 launch episode_recorder_app record_episode.launch.py \
  stream_config_uri:=/path/to/your_streams.yaml \
  root_dir:=$WORKSPACE_ROOT/data/episodes \
  experiment_name:=my_experiment \
  task:=my_task
```

Expect `[episode_recorder_app] /episode_recorder lifecycle active` shortly
after start. Confirm with:

```bash
ros2 lifecycle get /episode_recorder   # active
```

Then:

```bash
ros2 service call /episode_recorder/start_recording std_srvs/srv/Trigger
# ... run the episode ...
ros2 service call /episode_recorder/stop_recording std_srvs/srv/Trigger
ros2 service call /episode_recorder/get_status std_srvs/srv/Trigger
```

## Adding a new profile

Drop a stream-contract YAML into `config/<profile_name>.yaml` (same format
`episode_recorder` expects: a top-level `streams:` sequence — see
`config/marvin_teleop_8cam_streams.yaml` for a worked example) and launch with
`profile:=<profile_name>`. No code changes, no new package.

## Bundled profiles

| Profile | Description |
|---|---|
| `marvin_teleop_8cam_streams` | Marvin capture with the validated 8× Hik `test_camera_1..8` 640×480@60 profile, `/joint_states`, left/right EM status and pose references, `/tf`, `/tf_static`, and optional marker targets. Cameras and `/joint_states` form the start gate; both EM status streams are required at final validation but excluded from the 0.5 s freshness gate because they publish at 2 Hz. |

The launcher depends only on the generic recorder and ROS lifecycle/launch
packages. A profile's message packages must also be present at runtime. The
bundled Marvin profile requires `hik_camera_msg`, `geometry_msgs`,
`sensor_msgs`, `std_msgs`, and `tf2_msgs`; these are supplied by the Marvin/Hik
application workspace rather than imposed on unrelated recorder deployments.

## First hardware gate

This gate records real distributed cameras plus local Marvin fake hardware. It
does not publish a pose target and therefore commands no robot motion.

On the camera host, launch the already accepted 60 Hz profile:

```bash
ros2 launch hik_camera hik_camera.launch.py \
  config_file:=<hikvision_ros2>/hik_camera/config/multi_camera_60.yaml
```

On the recorder/controller host, use two terminals:

```bash
# Terminal 1: safe fake hardware, no RViz and no input source
ros2 launch marvin_controller_bringup controller_bringup.launch.py \
  use_fake_hardware:=true use_rviz:=false
```

```bash
# Terminal 2: the same generic launch, selecting only an application config
ros2 launch episode_recorder_app record_episode.launch.py \
  profile:=marvin_teleop_8cam_streams \
  root_dir:=/data/episodes \
  experiment_name:=marvin_hik_first_gate \
  task:=stationary_multimodal_capture \
  max_episode_duration:=10.0 \
  expected_max_bitrate_bytes_per_s:=180000000 \
  disk_reserve_bytes:=1073741824 \
  queue_capacity_messages:=8192 \
  queue_capacity_bytes:=536870912
```

After the lifecycle reports `active`, start one episode:

```bash
ros2 service call /episode_recorder/start_recording std_srvs/srv/Trigger
```

The recorder stops asynchronously at 10 seconds. Poll until `state` returns to
`idle`, then validate the finalized directory:

```bash
ros2 service call /episode_recorder/get_status std_srvs/srv/Trigger
ros2 run episode_recorder episode_validate \
  /data/episodes/marvin_hik_first_gate/episode_000000 \
  --verify-checksums
```

Pass requires validator `PASS`, zero recorder drops/writer failures, complete
eight-camera trigger groups, and valid required Marvin streams. Do not switch
to real Marvin hardware until this no-motion gate passes.

## Real-hardware 0.3.0 gate

After the no-motion gate, the same profile was accepted on 2026-07-23 with
real Marvin hardware, eight `640x480 bayer_rggb8 @ 60 Hz` Hikrobot cameras,
both execution managers, and local RViz marker teleoperation over CycloneDDS.
A manually bounded 10.632-second episode recorded 638 complete trigger groups
per camera, 5,317 joint states, and both raw-action and execution-command
streams with:

- exact `received == enqueued == written` accounting;
- zero recorder drops and no writer failure;
- 13.428 us maximum device timestamp spread;
- successful SHA-256 verification;
- `PASS` for `base_capture`, `recording_default`, and
  `marvin_8cam_imitation_learning`.

This result validates collection integrity and provenance only. Dataset
builders remain responsible for judging action content and selecting usable
samples.

## Launch arguments

| Argument | Default | Description |
|---|---|---|
| `profile` | `""` | Bundled stream-contract YAML name under `config/` (without `.yaml`); ignored when `stream_config_uri` is set |
| `stream_config_uri` | `""` | Absolute path to a stream-contract YAML; takes precedence over `profile` |
| `root_dir` | `data/episodes` | Episode storage root |
| `experiment_name` | `default` | Subdirectory under `root_dir` |
| `task` | `unspecified_task` | Manifest task string |
| `session_id` | `""` | Optional session id; empty lets the recorder assign one |
| `max_episode_duration` | `0.0` | Soft duration limit (0 = none) |
| `params_file` | `episode_recorder` default config | Node parameter YAML |
| `min_free_space_bytes` | `1073741824` | Refuse to arm below this much free disk |
| `disk_reserve_bytes` | `1073741824` | Extra disk reserve beyond planned footprint |
| `expected_max_bitrate_bytes_per_s` | `0` | Planned bitrate for disk preflight (0 disables) |
| `queue_capacity_messages` | `4096` | Bounded capture queue message capacity |
| `queue_capacity_bytes` | `1073741824` | Bounded capture queue byte capacity |

## Validate

```bash
ros2 run episode_recorder episode_validate \
  <root_dir>/<experiment_name>/episode_000000 --verify-checksums
```

## Scope

- Composition only: no recorder core, no LeRobot export.
- No robot, camera, or controller assumptions in code — only in the
  optional, swappable profile YAMLs.

## License

Apache-2.0. Composes `episode_recorder`; see that package for its license.
