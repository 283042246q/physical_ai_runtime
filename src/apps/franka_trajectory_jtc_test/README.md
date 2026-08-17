# franka_trajectory_jtc_test

Distributed MoveIt-style full-trajectory executor for one Franka FR3 arm:

```text
Operator PC                         RT / robot PC
───────────                         ─────────────
send_smooth_trajectory.py
move_to_start.py
  -> /action_sources/trajectory_test/joint_trajectory_goal
                                    manipulation_execution_manager
                                      -> /fr3_arm_controller/follow_joint_trajectory
                                    JointTrajectoryController (effort + PID)
                                      -> fake or real FR3
```

Same contract as `policy_inference/examples/diffusion_planner_example.py`
(`joint_trajectory_goal` → EM → FollowJointTrajectory), with FR3 joint names.
Controller profile matches vendor `franka_fr3_moveit_config` (`fr3_arm_controller`,
effort command interfaces + per-joint PID gains). MoveIt itself is **not** started.

## Roles

| Host | Launch / script |
|------|-----------------|
| **RT** | `trajectory_executor.launch.py` — FR3 + JTC + EM only |
| **Operator** | `send_smooth_trajectory.py` — small single-joint jog |
| **Operator** | `move_to_start.py` — smooth PTP to official Franka start pose |
| **Operator** | `joint_target_gui` — tkinter sliders + auto-send for joint-space targets |

Same `ROS_DOMAIN_ID` (default `1`) and CycloneDDS peers on both hosts
(see workspace `.config/cyclonedds_default.xml`). Person at e-stop on the robot host.

## Fake (single host or RT)

```bash
pixi run stop
source install/setup.bash
ros2 launch franka_trajectory_jtc_test trajectory_executor.launch.py \
  use_fake_hardware:=true
```

Expect `fr3_arm_controller` active:

```bash
ros2 control list_controllers
ros2 topic echo /franka/joint_states --once
```

In another terminal:

```bash
source install/setup.bash
ros2 run franka_trajectory_jtc_test send_smooth_trajectory
```

Expect logs like:

```text
Published smooth plan with 80 waypoints
PASS: JTC reported SUCCEEDED after smooth trajectory goal
```

API mirrors `policy_inference/examples/diffusion_planner_example.py`.
Exit policy matches `move_to_start.py` (wait for JTC SUCCEEDED, then exit).
The script also prints full trajectory positions/velocities before publish.

Re-run the script to send another trajectory; the RT executor stays up.

### Move to official start pose

Same executor, different operator script. Goal matches Franka's
`MoveToStartExampleController` / PTP example:

`q = {0, -π/4, 0, -3π/4, 0, π/2, π/4}`

```bash
ros2 run franka_trajectory_jtc_test move_to_start
# slower / explicit duration:
#   --ros-args -p speed_factor:=0.1
#   --ros-args -p duration_s:=8.0
```

Duration defaults from `speed_factor` (0.2, same idea as Franka `MotionGenerator`)
and MotionGenerator `dq_max`; set `duration_s > 0` to override.

## Joint-target GUI demo (EM → effort JTC)

Minimal interactive check of Franka torque JTC under streaming joint targets:

```text
joint_target_gui (tkinter sliders / auto-send)
  -> /action_sources/joint_gui/joint_trajectory_goal
  -> manipulation_execution_manager
  -> /fr3_arm_controller/follow_joint_trajectory
  -> JointTrajectoryController (effort + PID)
```

Each command publishes one short `JointTrajectory` segment (default `0.5` s).
EM routes `joint_trajectory_goal` to JTC.

### Launch split

| Launch | Role | Typical host |
|--------|------|--------------|
| `joint_gui_rt_bringup.launch.py` | FR3 + EM + effort JTC | RT / robot PC |
| `joint_gui_operator.launch.py` | Joint GUI or headless auto-send | Operator PC |
| `joint_gui_demo.launch.py` | All-in-one (both) | Single-host fake/debug |

### Distributed (RT PC + operator PC)

Same `ROS_DOMAIN_ID` on both machines. Cross-subnet DDS: use
`.config/cyclonedds_template.xml` (or site peers). Person at e-stop on the
robot host.

```bash
# RT / robot host — low-level only
pixi run stop
source install/setup.bash
ros2 launch franka_trajectory_jtc_test joint_gui_rt_bringup.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.2.101
```

Expect `fr3_arm_controller` active. Confirm on RT:

```bash
ros2 control list_controllers
ros2 topic echo /franka/joint_states --once
```

```bash
# Operator PC — GUI (after RT is up)
source install/setup.bash
ros2 launch franka_trajectory_jtc_test joint_gui_operator.launch.py
```

Sliders auto-sync from `/franka/joint_states` on connect. Enable **Auto-send**
to stream the current slider targets at `auto_send_rate_hz` (good JTC stress
test). Or use headless sine jog:

```bash
ros2 launch franka_trajectory_jtc_test joint_gui_operator.launch.py \
  operator_yaml:=$(ros2 pkg prefix franka_trajectory_jtc_test)/share/franka_trajectory_jtc_test/config/joint_gui_operator_auto_send.yaml
```

Cross-machine checks on the operator:

```bash
ros2 node list | grep -E 'robot_state_publisher|execution_manager|joint_target_gui'
ros2 topic echo /franka/joint_states --once
```

Stop both sides with `pixi run stop` before changing YAML or relaunching.

### A/B: impedance JSPC (Ruckig)

Distributed RT defaults to effort JTC. For joint impedance:

```bash
# RT
ros2 launch franka_trajectory_jtc_test joint_gui_rt_bringup.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.2.101 \
  controllers_yaml:=$(ros2 pkg prefix franka_trajectory_jtc_test)/share/franka_trajectory_jtc_test/config/controllers_jspc.yaml \
  execution_manager_yaml:=$(ros2 pkg prefix franka_trajectory_jtc_test)/share/franka_trajectory_jtc_test/config/execution_joint_gui_jspc.yaml \
  arm_controller:=franka_arm_jspc

# Operator
ros2 launch franka_trajectory_jtc_test joint_gui_operator.launch.py \
  operator_yaml:=$(ros2 pkg prefix franka_trajectory_jtc_test)/share/franka_trajectory_jtc_test/config/joint_gui_operator_jspc.yaml
```

`joint_gui_demo.launch.py` (single-host) defaults to this JSPC path.

### Fake / single host

```bash
pixi run stop
source install/setup.bash
ros2 launch franka_trajectory_jtc_test joint_gui_demo.launch.py
```

Drag any joint slider, or enable **Auto-send**.

### Operator parameters (`joint_gui_operator.yaml` = JTC)

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `output_contract` | `joint_trajectory_goal` | `joint_target` for JSPC |
| `joint_state_topic` | `/franka/joint_states` | Measured state (from RT host) |
| `goal_topic` | `/action_sources/joint_gui/joint_trajectory_goal` | EM JTC route |
| `move_duration_s` | `0.5` | Segment duration per JTC command |
| `publish_rate_hz` | `20.0` | Max manual publish rate |
| `use_gui` | `true` | `false` for headless auto-send |
| `sync_on_start` | `true` | Seed sliders from robot on connect |
| `auto_send` | `false` | Stream targets continuously |
| `auto_send_rate_hz` | `10.0` | Auto-send period |
| `auto_send_mode` | `stream` | `stream` (slider values) or `sine` |
| `auto_send_joint_index` | `3` | Joint for sine mode (`fr3_joint4`) |
| `auto_send_amplitude_rad` | `0.10` | Sine peak offset |
| `auto_send_period_s` | `4.0` | Sine period |

## Real hardware (distributed)

```bash
# RT
pixi run stop
source install/setup.bash
ros2 launch franka_trajectory_jtc_test trajectory_executor.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.2.101
```

```bash
# Operator (after RT is up)
source install/setup.bash
ros2 run franka_trajectory_jtc_test send_smooth_trajectory \
  --ros-args -p amplitude_rad:=0.15 -p duration_s:=5.0
```

Defaults move `fr3_joint4` by `0.25` rad out-and-back
over `4.0` s with a quintic blend. On real hardware start smaller if unsure.

## Useful parameters (`send_smooth_trajectory`)

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `joint_state_topic` | `/franka/joint_states` | Measured state |
| `goal_topic` | `/action_sources/trajectory_test/joint_trajectory_goal` | EM goal contract |
| `joint_index` | `3` | Which joint to displace (`fr3_joint4`) |
| `amplitude_rad` | `0.25` | Peak displacement |
| `duration_s` | `4.0` | Out-and-back duration |
| `num_points` | `80` | Trajectory samples |

## Diffusion / planner drop-in

Publish `trajectory_msgs/JointTrajectory` once to:

`/action_sources/trajectory_test/joint_trajectory_goal`

with `joint_names = fr3_joint1…fr3_joint7` and strictly increasing
`time_from_start`. Prefer non-zero velocities on interior points.

## MPD trajectory source

`send_mpd_trajectory` combines the isolated MPD runtime with this package's
FR3 EM-to-JTC route. Its public contract uses `fr3_joint1` through
`fr3_joint7`, while the current checkpoint retains its original
Panda-trained backend internally and still plans against
`EnvWarehouseExtraObjectsV00`.

The default goal input is Cartesian in `fr3_link0`, using ROS quaternion order
`[x, y, z, qx, qy, qz, qw]`. `infer_once.py` solves multiple IK candidates,
filters colliding candidates, and uses the selected joint solution for reachability
validation and as a compatibility reference for the legacy planner API. For the
current checkpoint, the actual diffusion context is the start joints plus the
requested EE pose.

Planning is non-commanding by default and saves `request.json`, `result.json`,
and `trajectory.npz` below `/tmp/mpd-fr3-plans/<request_id>`:

```bash
ros2 run franka_trajectory_jtc_test send_mpd_trajectory --ros-args \
  -p ee_pose_goal:="[0.4322543,0.1637504,0.6717085,0.8765521,0.4711762,0.0645563,-0.0740393]" \
  -p ik_candidates:=24 \
  -p ik_max_iters:=300
```

`ik_candidates` defaults to 0 (allowed range 0–256). Zero skips IK and uses
the measured `q_pos_start` as the internal legacy `q_pos_goal` placeholder. Positive
values enable IK. `ik_max_iters` defaults to 300 (allowed range 1–2000).

Joint input remains available explicitly:

```bash
ros2 run franka_trajectory_jtc_test send_mpd_trajectory --ros-args \
  -p goal_type:=joint \
  -p q_pos_goal:="[0.2,-0.3,0.1,-1.8,0.2,1.6,0.1]"
```

After either input mode finishes, the node prints `TARGET_CARTESIAN_POSE_XYZW`
and `BEST_TRAJECTORY_TERMINAL_CARTESIAN_POSE_XYZW`. Only the best trajectory
last-point pose is saved as `terminal_cartesian_pose_xyzw[7]` in
`trajectory.npz`; the full joint trajectory remains unchanged.

Fake-hardware execution test:

```bash
# Terminal 1
ros2 launch franka_trajectory_jtc_test trajectory_executor.launch.py \
  use_fake_hardware:=true

# Terminal 2
ros2 run franka_trajectory_jtc_test move_to_start
ros2 run franka_trajectory_jtc_test send_mpd_trajectory --ros-args \
  -p plan_only:=false \
  -p ee_pose_goal:="[0.4322543,0.1637504,0.6717085,0.8765521,0.4711762,0.0645563,-0.0740393]"
```

The MPD subprocess runs outside the ROS callback thread. Before publishing,
the node rechecks that the measured FR3 start state has not drifted from the
state used for planning. FR3-specific limit retiming is intentionally not
applied in this initial integration.

## Scope

- single FR3 arm, no gripper
- no MoveIt / RViz in this app
- fake hardware by default
