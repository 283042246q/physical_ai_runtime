"""Lightweight robot context interfaces for planner runtimes.

The context is a facade over robot state, model metadata, and execution
feedback. It deliberately starts small: concrete robot drivers or simulator
adapters can implement these protocols without forcing planner backends to
depend on ROS messages or controller-specific APIs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol

from .contracts import CurrentState
from .state_cache import RobotStateCache


@dataclass(frozen=True)
class RobotModelInfo:
    """Planner-relevant robot model metadata."""

    joint_names: list[str]
    base_frame: str = ""
    tool_frame: str = ""
    robot_description_xml: Optional[str] = None
    backend_config: dict | None = None


@dataclass(frozen=True)
class ExecutionFeedback:
    """Minimal execution state exposed to planner sources."""

    active: bool = False
    faulted: bool = False
    progress: Optional[float] = None
    message: str = ""


class RobotStateProvider(Protocol):
    """Provides latest name-matched robot state."""

    def get_current_state(self, now_s: float) -> Optional[CurrentState]:
        """Return a fresh state sample, or `None` if unavailable/stale."""
        ...


class RobotModelProvider(Protocol):
    """Provides robot model metadata needed by planner adapters."""

    def get_model_info(self) -> RobotModelInfo:
        """Return static or slowly-changing robot model metadata."""
        ...


class ExecutionFeedbackProvider(Protocol):
    """Provides downstream execution health/progress."""

    def get_execution_feedback(self) -> ExecutionFeedback:
        """Return current execution feedback if available."""
        ...


class RobotContext:
    """Small facade combining state, model, and feedback providers."""

    def __init__(
        self,
        *,
        state_provider: RobotStateProvider,
        model_provider: RobotModelProvider,
        feedback_provider: ExecutionFeedbackProvider | None = None,
    ) -> None:
        self._state_provider = state_provider
        self._model_provider = model_provider
        self._feedback_provider = feedback_provider or NullExecutionFeedbackProvider()

    def get_current_state(self, now_s: float) -> Optional[CurrentState]:
        return self._state_provider.get_current_state(now_s)

    def get_model_info(self) -> RobotModelInfo:
        return self._model_provider.get_model_info()

    def get_execution_feedback(self) -> ExecutionFeedback:
        return self._feedback_provider.get_execution_feedback()


class CachedJointStateProvider:
    """RobotStateProvider backed by `RobotStateCache`."""

    def __init__(self, joint_names: list[str], max_age_s: float) -> None:
        self.cache = RobotStateCache(joint_names, max_age_s)

    def update(
        self,
        msg_names: list[str],
        msg_positions: list[float],
        msg_velocities: Optional[list[float]],
        stamp_s: float,
    ) -> None:
        self.cache.update(msg_names, msg_positions, msg_velocities, stamp_s)

    def get_current_state(self, now_s: float) -> Optional[CurrentState]:
        return self.cache.get_fresh(now_s)

    @property
    def last_missing_joints(self) -> list[str]:
        return self.cache.last_missing_joints


class StaticRobotModelProvider:
    """RobotModelProvider for launch/config-supplied model metadata."""

    def __init__(self, model_info: RobotModelInfo) -> None:
        self._model_info = model_info

    def get_model_info(self) -> RobotModelInfo:
        return self._model_info


class NullExecutionFeedbackProvider:
    """Execution feedback provider for systems without downstream diagnostics."""

    def get_execution_feedback(self) -> ExecutionFeedback:
        return ExecutionFeedback()
