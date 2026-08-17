# Architecture

## Design objective

The Execution Manager (EM) is a robot-independent boundary between producers of
manipulation intent and consumers that execute position references. Its central
guarantee is that controller-facing topics receive only validated references
from the currently selected source and streaming route.

## Runtime data flow

```text
source topics
    |
    v
timestamp and payload validation
    |
    v
contract-specific normalization
    |
    v
per-source state and observability
    |
    v
global source + route arbitration
    |
    +--> JointTrajectory streaming output (JSPC route)
    +--> Pose/Twist streaming outputs (TSKPC route)
    `--> FollowJointTrajectory action client (independent goal route)
```

Incoming callbacks are event-driven. The node does not resample or intentionally
throttle references. A periodic timer checks freshness, and another publishes
status.

## Package modules

| Module | Responsibility |
|---|---|
| `execution_manager_node.py` | ROS parameters, subscriptions, publishers, timers, and action client |
| `execution_manager_core.py` | Contract constants, source state, and normalization primitives |
| `execution_manager_validation.py` | Timestamp acceptance and rejection policy |
| `execution_manager_ingest.py` | Contract dispatch and source-state mutation |
| `execution_manager_arbitration.py` | Winner selection and hold/preemption policy |
| `execution_manager_status.py` | Versioned status snapshot construction |
| `pose_utils.py` | Pure pose and quaternion helpers |

The pure modules are importable and unit tested independently of launch files.
ROS lifecycle and transport remain in the node modules.

## Source state

Each configured source has:

- integer priority;
- inactivity timeout;
- latest normalized joint, pose, and twist references;
- the route associated with its latest streaming reference;
- receipt timestamps and freshness state;
- validation, decode, normalization, and stale counters;
- its latest error string.

A source is eligible only while its most recent accepted reference remains
within both the global freshness window and its source-specific inactivity
timeout.

## Arbitration

The highest-priority eligible streaming source wins. Equal-priority candidates
are ordered by most recent receipt time. A configurable hold interval prevents
same-priority source flapping on the joint path; a higher priority source may
preempt during the hold.

Streaming routes are mutually exclusive:

- `jspc`: normalized joint targets and chunks;
- `tskpc`: pose targets, pose chunks, and twists.

Only the winning source's route may publish. Within the `tskpc` route, controller
configuration determines whether pose or twist is the active input mode.

Complete joint trajectories are different: they are forwarded to a
`FollowJointTrajectory` action server and do not enter streaming arbitration.

## Execution state

```text
IDLE -> ARMED -> ACTIVE -> DEGRADED
 ^         |        |          |
 |         `--------'          |
 `-----------------------------'
```

- `IDLE`: no accepted source reference is available.
- `ARMED`: a source has produced valid data but activation criteria are not yet
  satisfied.
- `ACTIVE`: an eligible source is selected and may publish.
- `DEGRADED`: previously active data became unavailable or stale; forwarding is
  stopped until recovery criteria are met.

`FAULT` is intentionally not part of this state machine. Fault enforcement and
emergency stop belong to the robot safety layer.

## Timing model

Message header stamps are compared with the node's ROS clock. Receipt time is
also recorded using the ROS clock for freshness and arbitration. This supports
simulation time and bag replay when all participants share the same ROS clock.

The EM does not estimate transport delay, synchronize sensors, or transform
between device clocks. Dataset alignment belongs to the recording pipeline.

## Safety boundary

The EM is a command gate, not a certified safety controller. Its protective
behavior is limited to rejecting malformed/stale references and suppressing
losing routes. Controller diagnostics and safety enforcement are intentionally
outside this package. See
[future_safety_monitor.md](future_safety_monitor.md) for the deferred design.
