# marvin_controller_bringup

Controller service and all-in-one RViz debug bringup for bimanual Marvin.

For distributed operation, the CPU host uses the marker-free controller
launch:

```bash
ros2 launch marvin_controller_bringup controller_bringup.launch.py
```

It starts:

```text
2x execution manager
  -> 2x TaskSpaceKinematicPositionController
  -> ros2_control hardware (fake by default; real Marvin SDK opt-in)
```

No RViz or interactive-marker node is defined or started by
`controller_bringup.launch.py`. The all-in-one local debug launch composes
that pure service with `rviz_marker_teleop profile:=marvin`:

```text
2x rviz interactive marker (source)
  -> 2x execution manager (validate / normalize / arbitrate, one per arm)
  -> 2x TaskSpaceKinematicPositionController (IK + joint position command)
  -> ros2_control hardware (fake by default; real Marvin SDK opt-in)
```

Fake hardware (`mock_components/GenericSystem`) is the default: no real
robot, no vendor SDK, no CAN/network I/O. Real hardware
(`marvin_hardware_interface/MarvinBimanualArmHardware`) is opt-in via
`use_fake_hardware:=false` -- see [Real hardware](#real-hardware) below.
Both arms (`Joint1_L`..`Joint7_L` and `Joint1_R`..`Joint7_R`) run through
independent, identically-shaped pipelines -- there is no coupling between
left and right beyond sharing one `controller_manager` process and one URDF.

## Run

```bash
ros2 launch marvin_controller_bringup rviz_debug_bringup.launch.py
```

Drag either interactive marker in RViz (`target_marker_left` seeded at
`flange_L`'s current pose, `target_marker_right` seeded at `flange_R`'s).
Each side's pose streams independently through:

1. `rviz_interactive_marker_pose_source` publishes `PoseStamped` on
   `/action_sources/marker_<side>/pose_target`.
2. `manipulation_execution_manager` (`em_left` / `em_right`) validates and
   forwards the winning route to
   `/position_controller/<side>_pose_reference`.
3. `manipulation_position_controllers/TaskSpaceKinematicPositionController`
   (`left_task_space_kinematic_position_controller` /
   `right_task_space_kinematic_position_controller`) solves IK
   (`osqp`) and writes position commands to that arm's 7 joints.
4. `mock_components/GenericSystem` fake hardware mirrors the commanded
   position back as state, closing the loop through `joint_state_broadcaster`
   and `robot_state_publisher` into TF/RViz.

Both TSKPC instances declare explicit `lower_limits` / `upper_limits` in
`config/controllers.yaml`, copied from
`marvin_description/config/joint_limits.yaml` (the same native M6 numbers
the URDF xacro loads). This duplication is intentional:
`TaskSpaceKinematicPositionController` does not parse the URDF's `<limit>`
tags itself -- it only clamps to `lower_limits`/`upper_limits` when given
explicitly, and defaults to `+-inf` (unlimited) otherwise. If the native
joint limits ever change, update both files.

## Distributed (CPU + local PC)

On the robot/CPU host keep EM + TSKPC + hardware; put markers and RViz on
the local PC via `rviz_marker_teleop`. Same `ROS_DOMAIN_ID` on both
hosts; for cross-machine DDS use `.config/cyclonedds_template.xml`.

```bash
# CPU
ros2 launch marvin_controller_bringup controller_bringup.launch.py

# Local PC
ros2 launch rviz_marker_teleop rviz_marker_teleop.launch.py profile:=marvin
```

## Launch arguments

| Argument | Default | Description |
|---|---|---|
| `use_rviz` | `true` | `rviz_debug_bringup` only: launch RViz with the Marvin profile |
| `use_markers` | `true` | `rviz_debug_bringup` only: launch both interactive-marker sources |
| `controllers_yaml` | `config/controllers.yaml` | `controller_manager` + both TSKPC instances' parameters |
| `use_fake_hardware` | `true` | `true`: `mock_components/GenericSystem`. `false`: real Marvin SDK bridge |
| `hardware_plugin` | `marvin_hardware_interface/MarvinBimanualArmHardware` | Real hardware plugin, used when `use_fake_hardware:=false` |
| `robot_ip` | `10.19.0.191` | Marvin controller IP, used when `use_fake_hardware:=false` |

Each arm's execution-manager profile
(`config/execution_manager_left.yaml` / `config/execution_manager_right.yaml`)
is fixed, not exposed as a launch argument -- they set the `em_left` /
`em_right` node names that this launch file's direct `Node(...)` calls
depend on.

## Real hardware

```bash
ros2 launch marvin_controller_bringup controller_bringup.launch.py \
    use_fake_hardware:=false robot_ip:=10.19.0.191
```

This loads `marvin_hardware_interface/MarvinBimanualArmHardware`, which opens
a TCP/IP SDK connection to the Marvin controller and can command real motor
motion on **both arms** as soon as their TSKPC controllers activate and a
pose reference arrives (from either marker or a manual topic publish). There
is no separate arm/disarm step in this bringup.

Only pass `use_fake_hardware:=false` when:

- the real robot is present, powered, and safed per its own runbook;
- someone is at the e-stop;
- the fake-hardware gate above has already been verified on this checkout.

Never use `use_fake_hardware:=false` as a default, in CI, or in an
unattended launch. `marvin_hardware_interface`'s own deterministic unit
tests (SDK bridge, write guard, transaction) run under `pixi run test`
without touching real hardware; this bringup's real-hardware path is not
covered by any automated test.

## Verify without RViz

```bash
ros2 launch marvin_controller_bringup controller_bringup.launch.py
ros2 control list_controllers
ros2 topic pub --once /action_sources/marker_left/pose_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: Base_L}, pose: {position: {x: 0.3, y: 0.2, z: 0.4}, orientation: {w: 1.0}}}"
ros2 topic pub --once /action_sources/marker_right/pose_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: Base_R}, pose: {position: {x: 0.3, y: -0.2, z: 0.4}, orientation: {w: 1.0}}}"
ros2 topic echo /execution_manager/left_status --once
ros2 topic echo /execution_manager/right_status --once
ros2 topic echo /joint_states --once
```

Expected: `left_task_space_kinematic_position_controller`,
`right_task_space_kinematic_position_controller`, and
`joint_state_broadcaster` are all `active`; both status topics report
`state: ACTIVE` with `active_streaming_route: tskpc`; both arms' joint
positions in `/joint_states` move toward their respective IK solutions.

## Scope

- Both arms, each through its own independent marker -> EM -> TSKPC chain.
  There is no coordinated bimanual motion (e.g. a shared task-frame or
  mirrored gesture) -- this is two single-arm pipelines sharing one
  `controller_manager` process and one URDF.
- Fake hardware by default. `marvin_hardware_interface` (the real SDK
  bridge) only loads when `use_fake_hardware:=false` is passed explicitly;
  see [Real hardware](#real-hardware).
- This package composes existing runtime packages; it owns no source code of
  its own beyond launch/parameter composition, per `docs/ARCHITECTURE.md`.

## Follow-ups (marker stack)

Workspace marker tooling was renamed/hardened:

- toolbox: `rviz_interactive_marker_teleop` → `rviz_interactive_marker_pose_source`
  (stable TF init; optional `required_joint_state_topic`;
  `publish_before_first_feedback` default off)
- app: `rviz_marker_teleop` Franka profile already wires those gates

This Marvin bringup still composes `rviz_marker_teleop profile:=marvin`. The
Marvin YAML only sets `publish_before_first_feedback: false` so far — it does
**not** yet mirror Franka's `required_joint_state_topic` / settled-TF policy
per arm. Before treating distributed or real-SDK teleop as validated:

1. Align `config` / docs here with the new pose-source contract (joint-state
   gate topics for left/right if Marvin publishes them).
2. Re-run fake smoke, then attended real-hardware marker teleop on Marvin.

## License

Apache-2.0. This package composes `marvin_description`,
`manipulation_execution_manager`, `manipulation_position_controllers`, and
`rviz_marker_teleop`; see each for its own license text.
