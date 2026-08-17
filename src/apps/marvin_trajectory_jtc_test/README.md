# Marvin trajectory -> EM -> JTC test

This app exercises the complete full-trajectory route on ROS 2 fake hardware:

```text
trajectory_chain_probe
  -> /action_sources/trajectory_test/joint_trajectory_goal
  -> manipulation_execution_manager
  -> /left_arm_controller/follow_joint_trajectory
  -> JointTrajectoryController
  -> mock_components/GenericSystem
  -> /joint_states + JTC action status
  -> trajectory_chain_probe PASS/FAIL
```

The app cannot select real hardware. It commands only the Marvin left arm on
`mock_components/GenericSystem`: `Joint1_L` moves from 0.00 rad to 0.10 rad and
back to 0.00 rad over 2.5 seconds.

## Run

```bash
pixi run build
pixi run bash -c 'source install/setup.bash && ros2 launch marvin_trajectory_jtc_test trajectory_em_jtc_test.launch.py'
```

Pass evidence is the line:

```text
PASS trajectory -> EM -> JTC: JTC succeeded; final max joint error=... rad
```

The probe fails after 10 seconds if startup is incomplete, or 8 seconds after
publishing if JTC never succeeds. The launch shuts down automatically after the
PASS or FAIL line.
