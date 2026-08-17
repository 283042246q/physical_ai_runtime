# manipulation_motion_planning

Backend-neutral motion-planner protocols and ROS source scaffolds.
Authoritative design: [`docs/MOTION_PLANNER_SOURCE_INTERFACE.md`](../../../../docs/MOTION_PLANNER_SOURCE_INTERFACE.md).
Environment/collision adapter design:
[`docs/MOTION_PLANNER_ADAPTER_WORLD_MODEL.md`](../../../../docs/MOTION_PLANNER_ADAPTER_WORLD_MODEL.md).

This package exports its public runtime API from
`manipulation_motion_planning/__init__.py`. The export list is intentionally
larger than a thin protocol package because planner adapters need a shared
runtime vocabulary: backend protocols, ROS source nodes, command output sinks,
robot context interfaces, and planner registry helpers.

## Three planner paths

| Path | Backend | Result types | EM → controller |
|---|---|---|---|
| Global setpoint | `GlobalSetpointBackend` | `SetpointPlanResult` | `joint_target` → JSPC |
| Global trajectory | `GlobalTrajectoryBackend` | `TrajectoryPlanPoint`, `TrajectoryPlanResult` | `joint_trajectory_goal` → JTC |
| Online horizon MPC | `OnlineMpcBackend` | `HorizonPlanPoint`, `HorizonPlanResult` | `joint_chunk` → JSPC |

Differential IK @ 500 Hz → **TSKPC** (not in this package).

## Public API Groups

### Backend Protocols

Backends are solver adapters. They do not import `rclpy` or ROS messages.

| Export | Meaning |
|---|---|
| `GlobalSetpointBackend` | Request-oriented global IK / goal-resolution backend. `plan()` returns one joint setpoint. |
| `GlobalTrajectoryBackend` | Request-oriented global trajectory backend. `plan()` returns a complete timed joint path. |
| `OnlineMpcBackend` | Online receding-horizon MPC backend. `step()` runs one solver step and returns a short horizon. |

Naming rule:

```text
global -> plan()
online MPC -> step()
ROS timer -> schedules the step, but is not the backend API
```

### Source Runtimes

Source runtimes own ROS subscriptions, validation, scheduling, diagnostics, and
command publication. They convert ROS inputs into backend-neutral contracts.

| Export | Meaning |
|---|---|
| `GlobalSetpointPlannerSource` | ROS node base for pose/joint target streams that publish `joint_target`. |
| `GlobalSetpointSourceConfig` | Config dataclass for `GlobalSetpointPlannerSource`. |
| `OnlineMpcPlannerSource` | ROS node base for online MPC sources that publish `joint_chunk`. |
| `OnlineMpcSourceConfig` | Config dataclass for `OnlineMpcPlannerSource`. |
| `GlobalTrajectoryPlannerRuntime` | Non-node helper for request-oriented global trajectory planning and publishing. |
| `PlannerSourceConfig` | Common source config fields: source name, namespace, state topic, sink mode. |

The source runtime keeps EM and controller contracts unchanged. The default
output path is still:

```text
planner source -> /action_sources/<source>/<contract> -> EM -> controller
```

### Command Sinks

A command sink is the output route for planner results. It separates the
planner runtime from the publication target.

| Export | Use |
|---|---|
| `EMCommandSink` | Production default. Publishes EM source contracts. |
| `DirectRobotCommandSink` | Debug/bring-up path. Publishes `JointTrajectory` directly to a configured topic. |
| `RecordingCommandSink` | Unit tests and diagnostics. Stores commands in memory. |
| `CommandSink` | Base interface for custom sinks. |
| `make_command_sink()` | Config-driven sink factory. |

`JointTrajectory` is a ROS message type, not an EM semantic contract. Both
`joint_chunk` and `joint_trajectory_goal` use `JointTrajectory`, but they remain
different contracts because they publish on different EM topics and have
different downstream semantics.

```text
joint_chunk             -> short horizon, refreshed by online MPC
joint_trajectory_goal   -> complete trajectory segment
```

### Robot Context

`RobotContext` is a lightweight facade over robot information. It is kept as an
interface layer so adapters can start simple and split responsibilities only
when needed.

| Export | Meaning |
|---|---|
| `RobotContext` | Facade combining state, model, and optional execution feedback. |
| `RobotStateProvider` | Interface for latest validated joint state. |
| `RobotModelProvider` | Interface for joint names, frames, robot description, backend config. |
| `ExecutionFeedbackProvider` | Interface for downstream execution health/progress. |
| `CachedJointStateProvider` | State provider backed by `RobotStateCache`. |
| `StaticRobotModelProvider` | Simple model provider for launch/config-supplied metadata. |
| `RobotModelInfo` | Dataclass for model metadata. |
| `ExecutionFeedback` | Dataclass for execution state. |

### World Contract

`World` is the backend-neutral collision and environment snapshot. Source nodes
or future environment providers can update it from static YAML, object services,
or perception output. Backends convert it to their native representation.

| Export | Meaning |
|---|---|
| `World` | One timestamped world snapshot in a planner frame. |
| `WorldPose` | Pose in xyz + wxyz form. |
| `WorldBox`, `WorldSphere`, `WorldCapsule` | Analytic collision primitives. |
| `WorldMesh` | Mesh URI plus pose/scale. |
| `WorldVoxelGrid` | Occupancy grid for dense collision input. |
| `WorldPointCloud` | Raw or filtered point cloud snapshot. |

PyRoki adapters should convert `World` to `pk.collision.CollGeom` lists. cuRobo
adapters should convert it to `WorldConfig`, `VoxelGrid`, or Blox input. Robot
collision spheres for cuRobo are robot model config, not runtime `World` input.

### Planner Registry

The registry/manager pair supports cuRobo-style cached planner switching
without forcing a specific backend implementation.

| Export | Meaning |
|---|---|
| `PlannerSpec` | Name + factory + display metadata for one planner. |
| `PlannerRegistry` | Registers planner factories. |
| `PlannerManager` | Lazily creates, caches, and switches active planner instances. |

## Typical Adapter Shape

```python
from manipulation_motion_planning import (
    GlobalSetpointPlannerSource,
    GlobalSetpointSourceConfig,
)


class MySetpointNode(GlobalSetpointPlannerSource):
    def __init__(self):
        super().__init__("my_setpoint_planner")
        backend = MyBackend(...)
        self.configure(
            backend=backend,
            state_joint_names=["j1", "j2"],
            output_joint_names=["j1", "j2"],
            config=GlobalSetpointSourceConfig(
                source_name="my_planner",
                command_sink_mode="em",
            ),
        )
```

For direct bring-up without EM:

```python
GlobalSetpointSourceConfig(
    source_name="my_planner",
    command_sink_mode="direct",
    direct_command_topic="/debug_joint_trajectory",
)
```

## Modules

| Module | Role |
|---|---|
| `contracts.py` | `Target`, `World`, world geometry, per-family result types |
| `backend.py` | Three backend `Protocol`s |
| `command_sink.py` | Planner output sinks: EM, direct `JointTrajectory`, recording |
| `planner_manager.py` | Planner registry/manager for cached runtime switching |
| `planner_sources.py` | Global setpoint, global trajectory dispatch, and online MPC ROS source runtimes |
| `robot_context.py` | Lightweight robot state/model/feedback facade |
| `state_cache.py` | `RobotStateCache` — latest `/joint_states` |
| `robot_description.py` | `resolve_robot_description_xml()` |

## Adapters (`src/toolbox/motion_planners/`)

| Package | Implements |
|---|---|
| `pyroki_planner_adapter` | `GlobalSetpointBackend` (J-PARSE), `GlobalTrajectoryBackend` (planned), `OnlineMpcBackend` (planned) |
| `curobo_global_planner_adapter` | `GlobalTrajectoryBackend` (planned) |
| `curobo_online_planner_adapter` | `OnlineMpcBackend` (planned) |

## Testing

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/toolbox/motion_planners/manipulation_motion_planning/test -v
```
