# pyroki_planner_adapter

Unified ROS adapter package for PyRoki planner backends.

PyRoki itself is a uv-managed Python dependency from the workspace
`pyproject.toml`. This ROS package is intentionally thin: it provides ROS
source nodes, package metadata, and command-line entry points.

## Backends

| Class | Protocol | Algorithm | Status |
|---|---|---|---|
| `PyrokiJparseSetpointBackend` | `GlobalSetpointBackend` | Iterative J-PARSE (`jparse_step`) | Implemented |
| `PyrokiTrajoptTrajectoryBackend` | `GlobalTrajectoryBackend` | `solve_trajopt` | Adapter shell |
| `PyrokiHorizonMpcBackend` | `OnlineMpcBackend` | `solve_online_planning` | Adapter shell |

## Layout

| Module | Role |
|---|---|
| `setpoint_node.py` | ROS source wrapper for J-PARSE setpoint planning |
| `mpc_node.py` | ROS source wrapper for online MPC |
| `pyroki_setpoint_backend.py` | `GlobalSetpointBackend` implementation |
| `pyroki_trajectory_backend.py` | `GlobalTrajectoryBackend` adapter shell |
| `pyroki_backend.py` | `OnlineMpcBackend` adapter shell |
| `world_adapter.py` | `World` -> `pk.collision.CollGeom` conversion |
| `robot_loader.py` | URDF -> `pk.Robot` / `RobotCollision` loading |
| `config.py` | Dataclass configs for nodes, robot loading, and solver shape |

## Entry Points

```bash
ros2 run pyroki_planner_adapter pyroki_global_setpoint_planner
ros2 run pyroki_planner_adapter pyroki_online_mpc_planner
```

`pyroki_global_setpoint_planner` is currently production-wired for Marvin IK
teleop. `pyroki_online_mpc_planner` starts a normal ROS source node, but the
backend still rejects `step()` until `solve_online_planning()` is wired.

