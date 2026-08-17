# Marvin Motion Planning Bringup

One Marvin app for independent left/right planning. Each enabled arm selects its
own planning contract, backend, target, and backend parameters in
`config/planning.yaml`.

```yaml
arms:
  left:
    enabled: true
    target:
      link_name: flange_L
      pose_topic: /teleop/pose_left
    planning:
      mode: online_mpc
      backend:
        name: pyroki
        parameters:
          step_rate_hz: 30.0
          horizon_steps: 5
```

Supported modes are `global_setpoint`, `global_trajectory`, and `online_mpc`.
The backend registry currently maps all three to `pyroki_planner_adapter`.
Unsupported mode/backend pairs fail before nodes are started.

The `arms` planners are independent, so left and right may use different modes.
They do not provide inter-arm collision or synchronization guarantees. The
reserved `groups.bimanual` schema is rejected while joint bimanual planner and
execution contracts are unavailable.

## Build

```bash
pixi run colcon build --symlink-install --packages-up-to marvin_motion_planning_bringup
```

The following demos use fake hardware and enable the left arm in their selected
planning profile. Run them from the workspace root.

## Global setpoint demo

The interactive marker continuously publishes the target after it initializes.
The planner emits joint setpoints through EM to the left JSPC.

```bash
pixi run bash -lc '
source install/setup.bash
ros2 launch marvin_motion_planning_bringup \
  marvin_motion_planning.launch.py \
  planning_config:=$PWD/install/marvin_motion_planning_bringup/share/marvin_motion_planning_bringup/config/global_setpoint.yaml \
  use_fake_hardware:=true
'
```

## Global trajectory demo

Drag the target marker, click **Plan**, inspect the path and pose axes in RViz,
then click **Execute**. This is the only mode that loads the Plan/Execute panel.

```bash
pixi run bash -lc '
source install/setup.bash
ros2 launch marvin_motion_planning_bringup \
  marvin_motion_planning.launch.py \
  planning_config:=$PWD/install/marvin_motion_planning_bringup/share/marvin_motion_planning_bringup/config/global_trajectory.yaml \
  use_fake_hardware:=true
'
```

## Online MPC demo

Wait for `PyRoki MPC solver ready`, then drag the target marker. The marker does
not publish a target before the first user pose update, so startup and solver
warmup cannot command motion by themselves.

```bash
pixi run bash -lc '
source install/setup.bash
ros2 launch marvin_motion_planning_bringup \
  marvin_motion_planning.launch.py \
  planning_config:=$PWD/install/marvin_motion_planning_bringup/share/marvin_motion_planning_bringup/config/online_mpc.yaml \
  use_fake_hardware:=true
'
```

To use another per-arm configuration, pass an absolute YAML path through
`planning_config:=/absolute/path/to/planning.yaml`.

The launch activates JTC for `global_trajectory` and JSPC for
`global_setpoint`/`online_mpc`; it never activates both command controllers for
the same arm.

Trajectory targets use the RViz Plan/Execute panel. Setpoint and MPC targets are
streamed by `rviz_interactive_marker_pose_source`.
