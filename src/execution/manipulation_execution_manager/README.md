# Manipulation Execution Manager

`manipulation_execution_manager` is a ROS 2 Jazzy package that places one
well-defined execution boundary between manipulation action sources and
position controllers.

It accepts references from teleoperation, motion planners, policies, or other
sources; validates and normalizes them; arbitrates between active sources; and
publishes only the winning streaming route.

```text
teleop / planner / policy
          |
          v
  Execution Manager
  validate -> normalize -> arbitrate -> observe
          |
          +--> joint streaming controller
          +--> task-space streaming controller
          `--> FollowJointTrajectory action server
```

## Scope

The package owns:

- timestamp, finite-value, joint-order, and configured-limit validation;
- source priority, freshness, takeover, and fallback;
- conversion of `JointState` targets into normalized `JointTrajectory` output;
- mutually exclusive joint-space and task-space streaming routes;
- independent forwarding of complete trajectories to a
  `FollowJointTrajectory` action server;
- versioned execution status for downstream diagnostics and recording.

It does not perform TF transforms, IK, collision checking, dynamics, hardware
I/O, controller switching, or emergency stop. Sources must publish commands in
the frame and units required by the selected controller.

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/Gabriel-Ning/manipulation_execution_manager.git
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-select manipulation_execution_manager
colcon test --packages-select manipulation_execution_manager
source install/setup.bash
```

The validated baseline is ROS 2 Jazzy with Python 3.12. When assembling a
custom Python environment, use `pytest>=8,<9`; pytest 9 is not compatible with
Jazzy's current `launch_testing` plugin.

## Quick start

Start the manager with the installed default profile:

```bash
ros2 launch manipulation_execution_manager execution_manager.launch.py
```

The default profile has no active sources, so startup is status-only and does
not publish motion references. A robot/application package should provide its
own parameter YAML and pass it explicitly:

```bash
ros2 launch manipulation_execution_manager execution_manager.launch.py \
  config_file:=/absolute/path/to/my_execution_manager.yaml
```

## Installed executables

| Executable | Purpose |
|---|---|
| `execution_manager` | Main validation, normalization, arbitration, and routing node |

## Documentation

- [Architecture](docs/architecture.md)
- [ROS contracts](docs/contracts.md)
- [Configuration](docs/configuration.md)
- [Future controller diagnostics and safety work](docs/future_safety_monitor.md)

## Safety

The execution manager stops forwarding references when no valid source wins.
It is not an emergency-stop implementation. Hardware limits, controller
safety, controller switching, and emergency-stop enforcement remain the
responsibility of the robot deployment.

## License

Apache-2.0. See [LICENSE](LICENSE).
