"""Backend protocol shapes.

See docs/MOTION_PLANNER_SOURCE_INTERFACE.md Section 7. Backends implement
these `Protocol`s structurally (no inheritance required) and must not import
`rclpy` or any ROS message type — the planner source node is the only layer
that touches ROS.

Global family (Section 6.1):
  - GlobalSetpointBackend  — single joint setpoint (global IK / goal resolution)
  - GlobalTrajectoryBackend — complete time-parameterized trajectory

Online family (Section 6.2):
  - OnlineMpcBackend — receding-horizon MPC only
"""

from __future__ import annotations

from typing import Optional, Protocol

from .contracts import (
    CurrentState,
    HorizonPlanResult,
    SetpointPlanResult,
    StartState,
    Target,
    TrajectoryPlanResult,
    World,
)


class GlobalSetpointBackend(Protocol):
    """Global setpoint backend (Section 6.1.1).

    Resolves a goal to a single joint-space configuration. Invoked at low
    frequency when the target changes; output is dispatched as EM
    `joint_target` → JSPC.
    """

    def warmup(self) -> None:
        """Pay JIT/model-load cost outside the request path."""
        ...

    def update_world(self, world: Optional[World]) -> None:
        """Update collision/planning-scene snapshot when the backend uses one."""
        ...

    def plan(
        self, start_state: StartState, target: Target, options: dict
    ) -> SetpointPlanResult:
        """Resolve goal to one setpoint, or return an explicit failure."""
        ...


class GlobalTrajectoryBackend(Protocol):
    """Global trajectory backend (Section 6.1.2).

    Produces a complete time-parameterized trajectory for one motion segment.
    Typically invoked once per segment; output is dispatched as EM
    `joint_trajectory_goal` → JTC.
    """

    def warmup(self) -> None:
        """Pay JIT/model-load/CUDA-init cost outside the request path."""
        ...

    def update_world(self, world: Optional[World]) -> None:
        """Update collision/planning-scene snapshot when the backend uses one."""
        ...

    def plan(
        self, start_state: StartState, target: Target, options: dict
    ) -> TrajectoryPlanResult:
        """Produce a complete trajectory, or an explicit failure. Never partial."""
        ...


class OnlineMpcBackend(Protocol):
    """Online horizon MPC backend (Section 6.2).

    Step-oriented receding-horizon optimization with trajectory warm start.
    Implementations must be gradient-based MPC (cost optimization over a
    horizon) or sampling-based MPC (MPPI / rollout / CEM). They must not use
    Jacobian single-step IK (jparse_step, J-PARSE, or equivalent).

    Output is dispatched as EM `joint_chunk` → JSPC. Source-node timer ticks
    schedule backend solver steps.
    """

    def warmup(self) -> None:
        """Pay JIT/model-load cost outside the step path."""
        ...

    def reset(self, current_state: CurrentState) -> None:
        """Re-seed trajectory warm start from a known-good state."""
        ...

    def update_target(self, target: Target) -> None:
        """Record the latest target. Does not trigger a solve by itself."""
        ...

    def update_world(self, world: Optional[World]) -> None:
        """Update collision/planning-scene snapshot when the backend uses one."""
        ...

    def step(self, current_state: CurrentState, dt: float) -> HorizonPlanResult:
        """Run one MPC solver step and return a horizon chunk."""
        ...
