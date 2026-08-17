# franka_controller_bringup

Physical AI Runtime controller bringup for one Franka FR3 arm:

```text
pose source
  -> manipulation_execution_manager
  -> TaskSpaceJointImpedanceController
       (Diff-IK @ HW rate + joint impedance)
  -> ros2_control effort command interfaces
  -> fake or real Franka hardware (torque / impedance on real FR3)
```

Prefer effort over Franka velocity/position motion generators.
A Diff-IK → velocity MG path was evaluated and is **not shipped**.
A future `TaskSpaceCartesianImpedanceController` may add true operational-space
/ Cartesian impedance; this bringup uses **joint** impedance after Diff-IK.

This app owns the runtime composition and its parameters. The vendor
`franka_bringup` package remains responsible for the FR3 description,
`robot_state_publisher`, controller manager, joint-state publication, and
Franka hardware plugin.

## Launch modes

| Launch | Role | Typical host |
|--------|------|--------------|
| `controller_bringup.launch.py` | Low-level: FR3 + EM + TSJI | RT / robot PC |
| `rviz_marker_teleop` (`profile:=franka`) | Marker + optional RViz | Operator PC |
| `rviz_debug_bringup.launch.py` | All-in-one (both) | Single-host fake/debug |

For a distributed robot/CPU service without UI:

```bash
ros2 launch franka_controller_bringup controller_bringup.launch.py
```

For an all-in-one local fake-hardware debug session:

```bash
ros2 launch franka_controller_bringup rviz_debug_bringup.launch.py
```

The debug entrypoint composes the same pure controller service with
`rviz_marker_teleop profile:=franka`. Both modes default to fake hardware.

## Distributed (RT PC + operator RViz)

Same `ROS_DOMAIN_ID=1` and CycloneDDS peers on both hosts
(`192.168.1.100` ↔ `192.168.1.113`). Person at e-stop on the robot host.

```bash
# RT / robot host — low-level only
pixi run stop   # or scripts/stop_ros.sh
source install/setup.bash
ros2 launch franka_controller_bringup controller_bringup.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.2.101
```

Expect `task_space_joint_impedance_controller` active. Confirm on RT:

```bash
ros2 control list_controllers
ros2 topic echo /franka/joint_states --once
```

```bash
# Operator PC — marker + RViz only (after RT is up)
source install/setup.bash
ros2 launch rviz_marker_teleop rviz_marker_teleop.launch.py \
  profile:=franka use_rviz:=true
```

Cross-machine checks on the operator:

```bash
ros2 node list | grep -E 'robot_state_publisher|execution_manager|target_marker'
ros2 topic echo /franka/joint_states --once
ros2 topic echo /execution_manager/status --once
```

Stop both sides with `pixi run stop` before changing YAML or relaunching.
Do **not** overwrite RT `.config/cyclonedds_default.xml` with the operator
PC copy (each host must bind its own `NetworkInterface`).

## Unified joint-controller contract

The Franka ros2_control description exports these interfaces for each
`fr3_joint1` through `fr3_joint7`:

- command: `position`, `velocity`, `effort`
- state: `position`, `velocity`, `effort`

TSJI claims `<joint>/effort` and reads joint `position`/`velocity` (plus
optional `fr3/robot_time`). EM forwards Pose to
`/task_space_joint_impedance_controller/pose_reference`.

## Fake-hardware gate

Fake hardware is the default and does not connect to a robot:

```bash
ros2 launch franka_controller_bringup controller_bringup.launch.py
```

Optional all-in-one UI:

```bash
ros2 launch franka_controller_bringup rviz_debug_bringup.launch.py
```

Confirm:

```bash
ros2 control list_controllers
ros2 topic echo /franka/joint_states --once
```

## Real-hardware gate

```bash
ros2 launch franka_controller_bringup controller_bringup.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.2.101
```

This opens the Franka connection and activates an effort-command controller.
A valid pose message can therefore cause real motion. Use this only with the
FR3 powered, unlocked, safed according to the site runbook, and with someone
at the emergency stop.

Stop with `pixi run stop` before changing YAML or relaunching.

### Baseline notes for TSJI testing

Production Diff-IK + joint impedance path:
- `robot_time_interface: fr3/robot_time`
- `solver.backend: osqp`
- Joint impedance gains from the JSIC FR3 scale (`kp_stiffness` / `kd_damping`)
- Gravity compensation assumed from the Franka hardware stack

TSKVC / velocity MG path is not shipped; use TSJI (effort).

## Scope

- single FR3 arm
- no gripper
- no namespace or arm prefix
- `controller_bringup.launch.py` starts no RViz or interactive-marker process
- `rviz_debug_bringup.launch.py` composes the Franka marker/RViz profile
- fake hardware by default

Other Franka models and multi-arm/prefixed deployments need model-specific
joint names, frames, and joint-limit profiles and are intentionally deferred.
