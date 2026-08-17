# ROS Contracts

This document defines the public wire contract for version `0.1.x`. Changes to
topic suffixes, message types, timestamp interpretation, arbitration semantics,
or status fields require a changelog entry and compatibility review.

## Source namespace

Inputs are created below:

```text
<source_namespace>/<source_name>/<contract>
```

The default source namespace is `/action_sources`. Source names and enabled
optional contracts are declared by the `sources` parameter.

## Input contracts

| Contract | Topic suffix | Message | Availability |
|---|---|---|---|
| Joint target | `joint_target` | `sensor_msgs/msg/JointState` | Always subscribed |
| Joint chunk | `joint_chunk` | `trajectory_msgs/msg/JointTrajectory` | Always subscribed |
| Pose target | `pose_target` | `geometry_msgs/msg/PoseStamped` | Enabled by `pose_contracts` |
| Pose chunk | `pose_chunk` | `geometry_msgs/msg/PoseArray` | Enabled by `pose_contracts` |
| Twist target | `twist_target` | `geometry_msgs/msg/TwistStamped` | Enabled by `twist_contracts` |
| Full joint trajectory | `joint_trajectory_goal` | `trajectory_msgs/msg/JointTrajectory` | Enabled by `goal_contracts` |

`JointState.name` is authoritative; consumers must not depend on source array
order. Pose and twist messages must already use the controller's required frame.
The EM does not call TF.

## Normalization

- Joint positions are reordered into configured `joint_names` order.
- A joint target becomes a single-point `JointTrajectory`.
- Joint chunks retain valid velocity and acceleration arrays after reordering.
- Missing joints and non-finite numeric values are rejected.
- Configured position limits are applied by the joint decoder.
- Pose/twist frames and finite values are validated but not transformed.

## Output contracts

| Route | Default endpoint | Type |
|---|---|---|
| Joint streaming | `/position_controller/joint_reference` | `trajectory_msgs/msg/JointTrajectory` |
| Pose streaming | `/position_controller/pose_reference` | `geometry_msgs/msg/PoseStamped` |
| Pose chunk streaming | `/position_controller/pose_chunk_reference` | `geometry_msgs/msg/PoseArray` |
| Twist streaming | `/position_controller/twist_reference` | `geometry_msgs/msg/TwistStamped` |
| Full trajectory | `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |

Joint and task-space streaming routes are mutually exclusive. Full-trajectory
goals use an independent action path and do not participate in that exclusion.
If the action server is unavailable, the goal is dropped with a throttled
warning rather than blocking the executor.

## Timestamp policy

| Condition | Default action |
|---|---|
| Zero stamp | Replace with current ROS time |
| Zero stamp with strict mode enabled | Reject |
| Older than `stale_timeout_s` | Reject |
| More than `max_future_s` in the future | Reject |

Accepted messages keep their semantic header stamp where the output type
supports it. Freshness and arbitration use node receipt state in the same ROS
clock domain.

## QoS

| Stream | Reliability | History | Durability |
|---|---|---|---|
| Reference input/output | Reliable | Keep last 1 | Volatile |
| Execution status | Reliable | Keep last 10 | Volatile |

Sources must offer QoS compatible with the reference subscription.

## Execution status

The node publishes JSON in `std_msgs/msg/String` on `status_topic`. The payload
contains `schema_version: 1` and the following stable top-level fields:

| Field | Meaning |
|---|---|
| `schema_version` | Status schema version |
| `state` | `IDLE`, `ARMED`, `ACTIVE`, or `DEGRADED` |
| `active_source` | Human-readable active source |
| `winning_source` | Joint-path winner or `null` |
| `streaming_winner` | Global streaming winner or `null` |
| `active_streaming_route` | `jspc`, `tskpc`, or `null` |
| `winning_source_priority` | Winner priority or `null` |
| `decode_fail_count` | Aggregate decoder failures |
| `normalize_drop_count` | Aggregate post-decode drops |
| `stale_count` | Aggregate stale rejections |
| `invalid_stamp_count` | Aggregate zero/future stamp rejections |
| `normalize_latency_ms` | Last winner normalization latency |
| `joint_names` | Configured output joint order |
| `sources` | Per-source state and counters |
| `last_error` | Latest winner error string |

Consumers must reject unknown future schema versions rather than assuming field
semantics remain unchanged.
