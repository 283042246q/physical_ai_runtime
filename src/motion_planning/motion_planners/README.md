# motion_planners

Backend-neutral motion-planner source contracts and solver adapters for
[Physical AI Runtime](https://github.com/Gabriel-Ning/physical_ai_runtime).

This monorepo contains three `ament_python` packages that land under
`src/motion_planning/motion_planners/` via `repos/necessary.repos`.

| Package | Role |
|---|---|
| `manipulation_motion_planning` | Backend-neutral protocols, ROS source scaffolds, EM command sinks |
| `pyroki_planner_adapter` | PyRoki / J-PARSE global setpoint (and MPC shell) adapter |
| `curobo_planner_adapter` | cuRobo online MPC adapter |

## Planner paths

| Path | Backend protocol | EM contract |
|---|---|---|
| Global setpoint | `GlobalSetpointBackend.plan()` | `joint_target` |
| Global trajectory | `GlobalTrajectoryBackend.plan()` | `joint_trajectory_goal` |
| Online MPC | `OnlineMpcBackend.step()` | `joint_chunk` |

Default output path:

```text
planner source -> /action_sources/<source>/<contract> -> manipulation_execution_manager -> controller
```

Solvers (`pyroki`, `nvidia-curobo`) are provided by the runtime Pixi environment,
not by this repository.

## Build / test (in Physical AI Runtime)

```bash
vcs import src < repos/necessary.repos
pixi run build
colcon test --packages-select manipulation_motion_planning --event-handlers console_direct+
```

See each package README for module details and entry points.
