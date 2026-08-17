# curobo_planner_adapter

ROS adapter package for NVIDIA cuRobo planner backends.

cuRobo, Torch, CUDA Python libraries, and Warp are uv-managed from the
workspace `pyproject.toml`. This ROS package only owns ROS source nodes,
backend adapters, and package metadata.

## Backends

| Class | Protocol | cuRobo API | Status |
|---|---|---|---|
| `CuroboMotionPlannerBackend` | `GlobalTrajectoryBackend` | `MotionPlanner.plan_pose()` / `plan_cspace()` | Wired |
| `CuroboMpcBackend` | `OnlineMpcBackend` | `ModelPredictiveControl.optimize_action_sequence()` | Wired |

## Robot Config

cuRobo expects a cuRobo robot config, not just a ROS URDF. For Marvin/Piper we
still need a prepared YAML with:

- kinematics and joint limits
- target/tool frame configuration
- robot collision spheres
- optional self-collision and scene settings

The default `robot` parameter is `franka.yml`, which comes from cuRobo's
packaged content and is useful for smoke tests.

## Entry Point

```bash
ros2 run curobo_planner_adapter curobo_online_mpc_planner
```

Important parameters:

| Parameter | Default | Meaning |
|---|---:|---|
| `robot` | `franka.yml` | cuRobo robot YAML name or config path |
| `scene_model` | empty | optional cuRobo scene YAML |
| `target_link_name` | empty | defaults to cuRobo `tool_frames[0]` |
| `pose_topic` | `/teleop/pose_commands` | `geometry_msgs/PoseStamped` target input |
| `source_name` | `curobo_mpc` | EM action source name |
| `command_sink_mode` | `em` | `em`, `direct`, or `recording` |
| `step_rate_hz` | `50.0` | scheduler tick rate |
| `optimization_dt` | `0.02` | cuRobo MPC knot dt |
| `horizon_points` | `1` | points published per EM `joint_chunk` |

With `horizon_points=1`, the adapter publishes one optimized action point per
step, matching the usual cuRobo MPC execution style. Raising it lets EM consume
a larger receding-horizon chunk.
