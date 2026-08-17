# Configuration

## Launch interface

```bash
ros2 launch manipulation_execution_manager execution_manager.launch.py \
  config_file:=/absolute/path/to/execution_manager.yaml \
  namespace:=robot \
  status_rate_hz:=2.0
```

Use an application-owned YAML for robot joint names, controllers, frames,
limits, and sources. The installed YAML is a safe status-only example.

## Core parameters

| Parameter | Default | Description |
|---|---:|---|
| `joint_names` | `joint_1,...,joint_6` | Required output joint order |
| `source_namespace` | `/action_sources` | Root namespace for source topics |
| `sources` | empty | JSON object encoded as a ROS string |
| `active_source` | `none` | Legacy single-source fallback when `sources` is empty |
| `output_topic` | `/position_controller/joint_reference` | Joint streaming output |
| `pose_output_topic` | `/position_controller/pose_reference` | Pose output |
| `pose_chunk_output_topic` | `/position_controller/pose_chunk_reference` | Pose chunk output |
| `twist_output_topic` | `/position_controller/twist_reference` | Twist output |
| `jtc_action_name` | `/arm_controller/follow_joint_trajectory` | Full trajectory action endpoint |
| `status_topic` | `/execution_manager/status` | Execution status topic |
| `status_rate_hz` | `2.0` | Status frequency |
| `stale_timeout_s` | `0.5` | Global freshness limit |
| `max_future_s` | `0.1` | Maximum accepted future offset |
| `reject_zero_stamped_references` | `false` | Reject rather than replace zero stamps |
| `arm_to_active_count` | `1` | Valid messages required for activation |
| `degraded_to_active_count` | `3` | Fresh checks required for recovery |
| `min_hold_duration_s` | `0.05` | Joint-path anti-flap hold |
| `joint_limits_lower` | empty | Comma-separated lower limits |
| `joint_limits_upper` | empty | Comma-separated upper limits |

Parameters are startup configuration. Runtime mutation is not a supported API.

## Source configuration

ROS parameters cannot represent arbitrary nested maps. `sources` is therefore
one JSON object encoded as a string:

```yaml
execution_manager:
  ros__parameters:
    joint_names: "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6"
    sources: >-
      {"teleop":{"priority":100,
                   "inactive_timeout_s":0.25,
                   "pose_contracts":["pose_target"],
                   "twist_contracts":["twist_target"]},
       "planner":{"priority":50,
                    "inactive_timeout_s":1.0,
                    "goal_contracts":["joint_trajectory_goal"]}}
```

Per-source keys:

| Key | Type | Meaning |
|---|---|---|
| `priority` | integer | Higher values win |
| `inactive_timeout_s` | positive number | Per-source eligibility timeout |
| `pose_contracts` | string or string array | Enable pose subscriptions |
| `twist_contracts` | string or string array | Enable `twist_target` |
| `goal_contracts` | string or string array | Enable `joint_trajectory_goal` |

Malformed JSON, invalid types, and non-positive timeouts fail node startup.
