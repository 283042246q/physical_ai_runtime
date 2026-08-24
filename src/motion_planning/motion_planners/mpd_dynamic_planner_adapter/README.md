# MPD Phase 4 dynamic planner adapter

This package is a separate dynamic-world entrypoint.  It depends on and reuses
the pure Phase-3 trajectory/coordinator/handoff utilities, but it does not alter
`mpd_planner_adapter/replan.launch.py` or its static worker contract.

The package contains the constant-velocity Kalman filter, versioned world
snapshots, fixed-timing collision-sphere validation, earliest safe low-speed
handoff selection, bounded braking trajectory generation, and the independent
ROS node/launch entry.

The MPD worker is external and must be started from the MPD repository first:

```bash
conda run --no-capture-output -n mpd-splines-public \
  python scripts/runtime/infer_dynamic_server.py \
  --socket /tmp/mpd-dynamic-runtime.sock \
  --output-root /tmp/mpd-dynamic-results \
  --device cuda:0
```

Plan-only ROS validation with a direct target and the deterministic far-away
known-object publisher:

```bash
cd /home/eric/Projects/physical_ai_runtime
pixi run bash -lc '
source install/setup.bash
ros2 run mpd_dynamic_planner_adapter dynamic_world_demo \
  --ros-args -p scenario:=safe_far &
ros2 launch mpd_dynamic_planner_adapter replan_dynamic.launch.py \
  plan_only:=true \
  plan_rate_hz:=0.5 \
  target_pose_xyzw:="0.4322543,0.1637504,0.6717085,0.4711762,0.0645563,-0.0740393,0.8765521"
'
```

Full fake-hardware rehearsal uses the production Franka server and activates
`franka_arm_jtc` explicitly:

```bash
pixi run bash -lc '
source install/setup.bash
ros2 launch mpd_dynamic_planner_adapter \
  replan_dynamic_fake_hardware.launch.py \
  plan_only:=false \
  world_scenario:=safe_far
'
```

For a non-executing multi-cycle check, change the same launch argument to
`plan_only:=true`. In that mode every cycle starts from measured JointState and
publishes only the accepted result; an unexecuted result is deliberately not
stored as the active trajectory.

Observation topic: `/mpd/dynamic_world_observations` (`std_msgs/msg/String`).
Each JSON message is a complete known-object snapshot in `fr3_link0`; absent
objects become inactive. The node estimates `[position, velocity]` with one CV
Kalman filter per object and uploads an immutable, monotonically versioned
snapshot. Future orientation is held constant. The default process acceleration
standard deviation is `0.01 m/s^2`; it is only a fake-hardware-safe starting
point and must be calibrated from the deployed tracker's residuals. Inflation
can use propagated covariance or a configured linear horizon envelope.

Safety behavior:

- every active trajectory is checked at 20 Hz against the newest world using
  MPD-exported collision sphere positions and absolute trajectory time;
- a result is checked again against one stable latest world version immediately
  before JTC submission, including the old prefix and new suffix;
- handoff search chooses the earliest dynamically safe point below the configured
  joint-speed threshold;
- no feasible handoff, stale world, or guard collision sends a bounded braking
  trajectory and clears the target;
- `/mpd/emergency_stop` is latched in the dynamic node; call
  `/mpd_dynamic_replanner/reset_stop` and publish a new target to resume.

This is a soft-real-time geometric safety layer, not a certified protective
stop. Real-hardware use still requires the robot's independent protective stop,
site-specific braking calibration, and conservative speed/acceleration limits.

## Phase 5 candidate-specific timing entry

Phase 4 remains available through the commands above. Phase 5 uses a separate
worker socket, ROS executable, config, and launch file. Start the inference-only
space-time worker from the MPD repository:

```bash
conda run --no-capture-output -n mpd-splines-public \
  python scripts/runtime/infer_space_time_server.py \
  --socket /tmp/mpd-space-time-runtime.sock \
  --output-root /tmp/mpd-space-time-results \
  --device cuda:0 \
  --timing-mode phase5_joint
```

Then run the ROS fake-hardware closed loop:

```bash
pixi run bash -lc '
source install/setup.bash
ros2 launch mpd_dynamic_planner_adapter \
  replan_space_time_fake_hardware.launch.py \
  plan_only:=false \
  world_scenario:=safe_far \
  timing_mode:=phase5_joint
'
```

The Phase-5 backend requires trajectory artifact schema v3 and timing schema
v1. Every top-K candidate carries its own strictly increasing time array; the
quintic bridge, common-window comparison, collision guard, and latest-world
revalidation consume those candidate timestamps directly. The launch also
accepts `phase5_scalar_duration` and `phase5_timing_only` when the worker is
started in the matching mode.

## Passive replay recording

`replan_dynamic.launch.py` and `replan_dynamic_fake_hardware.launch.py` retain their old
defaults. Recording is enabled only when all three output arguments are supplied:

```bash
replay_record_dir:=/tmp/mpd-episode \
replay_env_name:=EnvOpenDrawerShelf \
replay_static_scene_json:=/tmp/static-scene.json
```

The recorder is an observer: it cannot approve/reject a plan or change the command sent
to JTC. Runtime write failures disable recording and leave planning/execution running.
It atomically checkpoints `replay-manifest.json` after planning/execution state changes,
so a second interrupt during ROS shutdown does not lose the episode. The additional
`scene_id` and `socket_path` launch arguments allow a selectable MPD environment and a
run-local resident-worker socket without changing the original defaults.

For the complete ToDrawer fake-hardware-to-video command, use
`mpd/scripts/isaaclab/run_dynamic_demo_pipeline.sh --profile to_drawer` from the MPD
repository. The pipeline defaults to Phase 5 `phase5_joint`; select the preserved
fixed-timing path with `--phase phase4`, or choose a Phase 5 ablation with
`--phase phase5 --timing-mode phase5_timing_only` (or
`phase5_scalar_duration`).
