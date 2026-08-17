# MPD planner adapter

ROS 2 adapter for the resident MPD Unix-domain-socket worker.  CUDA inference
stays in the MPD Conda environment; this package only performs bounded IPC,
generation/deadline checks, trajectory conversion, and ROS orchestration.

Phase 2 runs with `plan_only:=true` by default.  It subscribes to
`/franka/joint_states`, `~/pose_target` (`PoseStamped`), `~/joint_target`
(`JointState`), and `~/stop` (`Bool`), and publishes `~/planned_trajectory` plus
JSON diagnostics on `~/diagnostics`.

`~/world_version` (`UInt64`) invalidates in-flight work when a newer static
scene snapshot is announced.  Phase 2 intentionally rejects geometry updates;
dynamic objects start in Phase 4.

The production Franka server contract is:

- state: `/franka/joint_states`;
- owned trajectory action: `/franka_arm_jtc/follow_joint_trajectory`;
- the server must be launched with `activate_trajectory_controller:=true`
  while MPD owns that action.

The worker and replanner must share a host because `/tmp/mpd-runtime.sock` is a
Unix-domain socket. The Franka server may run on another host through ROS 2 DDS.

## Build

```bash
cd /home/eric/Projects/physical_ai_runtime
pixi run colcon build --packages-up-to \
  franka_manipulation_controller_bringup mpd_planner_adapter \
  --symlink-install --executor sequential
source install/setup.bash
```

## Start the resident MPD worker

```bash
cd /home/eric/Projects/MotionPlanningDiffusion/mpd
conda run --no-capture-output -n mpd-splines-public \
  python scripts/runtime/infer_server.py \
  --socket /tmp/mpd-runtime.sock \
  --output-root /tmp/mpd-runtime-results \
  --device cuda:0
```

Wait for `READY` before starting the replanner.

## Single-host fake hardware

With the worker running in another terminal:

```bash
cd /home/eric/Projects/physical_ai_runtime
pixi run bash -lc '
source install/setup.bash
ros2 launch mpd_planner_adapter replan_fake_hardware.launch.py \
  plan_only:=true plan_rate_hz:=1.0
'
```

This launch composes `franka_manipulation_controller_bringup`, activates
`franka_arm_jtc`, injects the Warehouse smoke-test target, and starts the MPD
replanner and independent cancel-all node. Set `plan_only:=false` only after the
plan-only diagnostics are healthy.

## Split-host server and planner

On the robot/controller host:

```bash
ros2 launch franka_manipulation_controller_bringup \
  controller_bringup.launch.py \
  use_fake_hardware:=true \
  activate_trajectory_controller:=true
```

On the GPU/planner host, after starting the resident worker:

```bash
cd /home/eric/Projects/physical_ai_runtime
pixi run bash -lc '
source install/setup.bash
ros2 launch mpd_planner_adapter replan.launch.py \
  plan_only:=true plan_rate_hz:=1.0
'
```

`replan.launch.py` has no implicit target. Publish a target only after checking
the remote state and action contracts. A joint target example is:

```bash
ros2 topic pub --once /mpd_replanner/joint_target sensor_msgs/msg/JointState \
  "{name: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], position: [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]}"
```

Before enabling execution, verify:

```bash
ros2 topic echo /franka/joint_states --once
ros2 action info /franka_arm_jtc/follow_joint_trajectory
ros2 control list_controllers
ros2 topic echo /mpd_replanner/diagnostics std_msgs/msg/String \
  --field data --once
```

`franka_arm_jtc` must be `active`. Both hosts must use the same ROS domain/DDS
configuration and synchronized system clocks.

Phase 3 execution is opt-in with `plan_only:=false`.  The node owns accepted
JTC goal handles, publishes lifecycle fields in diagnostics, and only replaces
an active goal after old-prefix/new-suffix continuity and low-speed gates pass.
`/mpd_replanner/safe_stop` invalidates planner work and cancels the owned goal.
The separately launched `/mpd_jtc_safe_stop/stop` service sends an action-level
cancel-all request and latches `/mpd/emergency_stop`; it remains available if
the replanner/MPD worker fails, and a restarted replanner receives the latch.
Neither interface replaces the robot's external protective-stop/E-stop layer.

The node never blocks a ROS callback on inference.  It has one active request
and one replaceable pending slot; superseded or stale responses are not
published or executed.
