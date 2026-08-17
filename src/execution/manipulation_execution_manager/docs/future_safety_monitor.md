# Future Controller Diagnostics and Safety Work

## Current decision

A development-stage `SafetyMonitor` was removed from this package. It was
useful for debugging controller integration, but it did not perform any state
transition, controller stop, goal cancellation, hardware inhibit, or
emergency-stop action.

The prototype only:

- decoded controller-specific `Float64MultiArray` status layouts;
- observed stale and IK-failure controller states;
- compared commanded and measured joint positions;
- logged threshold violations;
- published JSON diagnostics and optionally wrote a summary file.

Calling this component a safety monitor overstated its authority. An alert did
not mean that motion had stopped.

## Why it is outside the Execution Manager

The Execution Manager is robot- and controller-independent. The prototype was
tightly coupled to private JSPC/TSKPC array indices and controller topic names.
Keeping it here would make the generic execution contract depend on one
controller implementation and could silently misinterpret data after a
controller status layout changes.

Twist adaptation has the same ownership issue. Conversion from an unstamped
teleop `Twist` to a framed `TwistStamped` belongs to the teleop/source adapter.
That adapter must publish to an EM source topic such as
`/action_sources/<name>/twist_target`; it must not publish directly to a
controller reference topic and bypass EM arbitration.

## TODO: controller diagnostic watchdog

If controller diagnostics are required, create a separate controller
diagnostics package with:

1. Typed ROS status messages instead of positional `Float64MultiArray` fields.
2. Explicit controller identity and status schema version.
3. Independent freshness and alert state per controller.
4. Joint-name-based command/feedback comparison.
5. `diagnostic_msgs/msg/DiagnosticArray` or another documented typed output.
6. Defined severity, latching, acknowledgement, and reset semantics.
7. Replayable tests for stale input, tracking divergence, and IK failures.

This watchdog may run alongside the EM, but its initial implementation should
remain diagnostic-only and should be named accordingly, for example
`ManipulationControllerDiagnostics`.

## TODO: state-changing safety response

Actual safety response requires a separate, explicitly authorized design.
Before implementation, define:

- which component owns stop authority;
- how EM streaming output is inhibited;
- how active trajectory goals are cancelled;
- whether controllers are deactivated through `controller_manager`;
- how hardware quick-stop or emergency-stop is triggered;
- acknowledgement, reset, and recovery rules;
- behavior when ROS communication fails;
- real-time and functional-safety requirements.

Do not infer safe stop from a diagnostic topic. Hardware protection and
emergency-stop paths must remain effective independently of this Python
runtime.
