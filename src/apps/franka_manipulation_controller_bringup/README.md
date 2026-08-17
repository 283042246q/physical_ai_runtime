# franka_manipulation_controller_bringup

Production manipulation server for one Franka FR3 arm:

```text
command sources over DDS
  -> manipulation_execution_manager
  -> normalized controller command topics / JTC action
       joint_servo          -> Franka joint-space impedance (effort)
       cartesian_servo      -> Franka task-space joint impedance (effort)
       trajectory_execution -> Franka joint trajectory controller (effort)
  -> franka_bringup / ros2_control
```

This package is the only owner of Franka hardware, the three manipulation
controllers, and EM. UI, planning, policy, and probe processes run separately.

By default all three route controllers are loaded inactive. In the current
workspace EM arbitrates and forwards commands, but does not switch
`ros2_control` controllers. The selected route therefore has to be activated
explicitly. The launch file currently provides an explicit JTC activation mode
for an external trajectory owner; servo-route activation remains an operator
or future integration responsibility.

An external client that owns the complete `FollowJointTrajectory` lifecycle
(for example the MPD Phase-3 replanner) can explicitly start with the trajectory
controller active:

```bash
ros2 launch franka_manipulation_controller_bringup \
  controller_bringup.launch.py use_fake_hardware:=true \
  activate_trajectory_controller:=true
```

This opt-in exposes `/franka_arm_jtc/follow_joint_trajectory` immediately. Do
not run another trajectory source or activate another effort controller while
the MPD replanner owns this route. The default remains `false`.

## Fake-hardware gate

```bash
ros2 launch franka_manipulation_controller_bringup \
  controller_bringup.launch.py
```

Confirm that all three routes are present and inactive:

```bash
ros2 control list_controllers
ros2 topic echo /execution_manager/status --once
ros2 topic echo /franka/joint_states --once
```

## Real-hardware gate

On a CPU / RT host configured per
[docs/CPU_HOST_SETUP.md](../../../docs/CPU_HOST_SETUP.md), launch without a
manual `taskset` — bringup pins `ros2_control_node` to `RT_CM_CPU_AFFINITY`
(default `14,15`):

```bash
ros2 launch franka_manipulation_controller_bringup \
  controller_bringup.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.2.101
```

Override or disable pinning with `cpu_affinity:=12,13` or `cpu_affinity:=none`.

Use real hardware only with the FR3 powered, unlocked, safed according to the
site runbook, and with someone at the emergency stop. Starting the default
server does not activate a motion controller. Passing
`activate_trajectory_controller:=true` does activate JTC and an accepted action
goal can then cause motion.

Run operator-side examples from `franka_motion_demos`.

## Stable contracts

| EM route | Controller | Command interface |
|---|---|---|
| `joint_servo` | `franka_arm_jsic` | effort |
| `cartesian_servo` | `franka_arm_tsjic` | effort |
| `trajectory_execution` | `franka_arm_jtc` | effort |

Semantic controller topics:

- `/execution_manager/franka_arm/joint_position_reference`
- `/execution_manager/franka_arm/cartesian_pose_reference`
- `/execution_manager/franka_arm/cartesian_pose_chunk_reference`
- `/execution_manager/franka_arm/cartesian_twist_reference`
- `/franka_arm_jtc/follow_joint_trajectory`

The route names are normalized EM contracts. Controller names describe the
Franka-specific effort implementation.

## Scope

- single FR3 arm
- no gripper
- effort interface only
- fake hardware by default
- no RViz, marker, planner, policy, or test source in the server process

`robot_time_interface: fr3/robot_time` remains enabled for real hardware and
is disabled only by the fake-hardware override.
