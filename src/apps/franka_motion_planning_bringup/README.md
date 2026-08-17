# Franka Motion Planning Bringup

Distributed marker teleop validation for FR3:

```text
RViz marker
  -> PyRoki global setpoint planner (jparse)
  -> execution manager
  -> JointSpaceImpedanceController (Ruckig reference shaping)
  -> Franka effort interface
```

Kept separate from `franka_controller_bringup` for now; a future merge may
unify the two app families.

## Validated defaults (2026-07-26)

| Layer | Parameter | Value |
|-------|-----------|-------|
| Controller | `reference_behavior.mode` | `ruckig` |
| Controller | `max_velocity_rad_s` | `2.5` |
| Controller | `max_acceleration_rad_s2` | `10.0` |
| Controller | `max_jerk_rad_s3` | `150.0` |
| Controller | `kp_stiffness` (J1–J7) | `1000/1000/1000/1000/500/320/150` |
| Planner | `position_gain` / `orientation_gain` | `15.0` / `3.0` |
| Planner | `max_joint_velocity` | `2.5` |
| Planner | `max_step_rad` | `0.05` at `50 Hz` |
| Planner | `max_iterations_per_tick` | `4` |

Ruckig felt smoother than limiter+EMA. Planner `15/2.5` was OK under Ruckig;
the same planner step triggered `cartesian_reflex` on limiter+EMA.

## Launch split (same pattern as controller bringup)

| Launch | Role | Typical host |
|--------|------|--------------|
| `planning_bringup.launch.py` | Low-level: FR3 + EM + impedance(Ruckig) | RT / robot PC |
| `operator_bringup.launch.py` | High-level: marker + PyRoki + optional RViz | Operator PC |
| `franka_motion_planning.launch.py` | All-in-one (both) | Single-host fake/debug |

## Distributed (RT PC + operator PC)

Same `ROS_DOMAIN_ID` on both machines. Cross-subnet DDS: use
`.config/cyclonedds_template.xml` (or site peers). Person at e-stop on the
robot host.

```bash
# RT / robot host — low-level only
pixi run stop
source install/setup.bash
ros2 launch franka_motion_planning_bringup planning_bringup.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.2.101
```

Expect `franka_arm_jspc` active with `mode=ruckig`. Confirm:

```bash
ros2 control list_controllers
ros2 topic echo /franka/joint_states --once
```

```bash
# Operator PC — high-level only (after robot host is up)
source install/setup.bash
ros2 launch franka_motion_planning_bringup operator_bringup.launch.py \
  use_rviz:=true
```

Cross-machine checks on the operator:

```bash
ros2 node list | grep -E 'robot_state_publisher|execution_manager|motion_planner'
ros2 topic echo /franka/joint_states --once
```

Stop both sides with `pixi run stop` before changing YAML or relaunching.

## Fake / single-host all-in-one

```bash
pixi run bash -lc '
source install/setup.bash
ros2 launch franka_motion_planning_bringup \
  franka_motion_planning.launch.py use_fake_hardware:=true
'
```
