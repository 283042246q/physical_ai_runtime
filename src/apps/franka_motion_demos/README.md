# franka_motion_demos

Four lightweight command-source demos for an already running
`franka_manipulation_controller_bringup` server. They never start Franka
hardware, ros2_control controllers, or an execution manager.

Start the server first:

```bash
ros2 launch franka_manipulation_controller_bringup \
  controller_bringup.launch.py use_fake_hardware:=true
```

For real hardware, use `use_fake_hardware:=false` only with the FR3 present,
powered, and attended.

## Global IK to JSIC

```bash
ros2 launch franka_motion_demos global_ik_demo.launch.py
```

The RViz target feeds PyRoki global setpoint IK. Joint setpoints are published
under `/action_sources/motion_planner/arm/joint_reference`; EM selects the
`joint_servo` route and activates the Franka joint-space impedance controller.

## Task-space marker to TSJIC

```bash
ros2 launch franka_motion_demos task_space_marker_demo.launch.py
```

The marker publishes `/action_sources/marker/arm/cartesian_pose`; EM selects the
`cartesian_servo` route and activates the task-space joint-impedance controller.

## Smooth trajectory through effort JTC

```bash
ros2 launch franka_motion_demos jtc_probe_demo.launch.py
```

The demo reproduces the previous quintic smooth-trajectory test: it reads the
current joint state, moves one joint out and back over four seconds, and
waits for the EM `joint_trajectory` action to report its terminal result.

## Move to Franka start pose

```bash
ros2 launch franka_motion_demos move_to_start_demo.launch.py duration_s:=4.0
```

This reads the current joint state and generates a four-second quintic trajectory
to Franka's official start configuration:
`[0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]`. EM selects the
`trajectory_execution` route and the
effort JTC executes the trajectory with zero endpoint velocity and
acceleration. The demo passes only after the action reports `SUCCEEDED` and
the measured joints settle within the configured goal tolerance.

## MPD trajectory

The migrated MPD client defaults to planning and validation only:

```bash
ros2 run franka_motion_demos send_mpd_trajectory.py
```

It prints the Cartesian target and the best trajectory's terminal Cartesian
pose. To submit a validated plan through the same EM trajectory action used by
the smooth demo, explicitly set `plan_only` to false:

```bash
ros2 run franka_motion_demos send_mpd_trajectory.py --ros-args \
  -p plan_only:=false
```
