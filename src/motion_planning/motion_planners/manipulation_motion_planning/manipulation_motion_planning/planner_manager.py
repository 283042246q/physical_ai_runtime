"""Small planner registry and manager.

This mirrors the useful part of curobo_ros' factory/manager pattern while
keeping backend implementations ROS-free. Factories are ordinary callables so
adapters can decide whether they return setpoint, trajectory, or online
backends.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Generic, Optional, TypeVar


BackendT = TypeVar("BackendT")
PlannerFactory = Callable[[], BackendT]


@dataclass(frozen=True)
class PlannerSpec(Generic[BackendT]):
    """Registered planner construction metadata."""

    name: str
    factory: PlannerFactory[BackendT]
    display_name: str = ""
    warmup_on_create: bool = True


class PlannerRegistry(Generic[BackendT]):
    """Maps planner names to backend factories."""

    def __init__(self) -> None:
        self._specs: dict[str, PlannerSpec[BackendT]] = {}

    def register(self, spec: PlannerSpec[BackendT]) -> None:
        key = spec.name.strip().lower()
        if not key:
            raise ValueError("planner name must not be empty")
        if key in self._specs:
            raise ValueError(f"planner already registered: {key}")
        self._specs[key] = spec

    def create(self, name: str) -> BackendT:
        spec = self.get_spec(name)
        backend = spec.factory()
        if spec.warmup_on_create and hasattr(backend, "warmup"):
            backend.warmup()
        return backend

    def get_spec(self, name: str) -> PlannerSpec[BackendT]:
        key = name.strip().lower()
        try:
            return self._specs[key]
        except KeyError as exc:
            available = ", ".join(self.available())
            raise ValueError(
                f"Unknown planner {name!r}. Available planners: {available}"
            ) from exc

    def available(self) -> list[str]:
        return sorted(self._specs)


class PlannerManager(Generic[BackendT]):
    """Caches planner instances and tracks the active planner."""

    def __init__(self, registry: PlannerRegistry[BackendT]) -> None:
        self._registry = registry
        self._instances: dict[str, BackendT] = {}
        self._active_name: Optional[str] = None

    def get(self, name: str) -> BackendT:
        key = name.strip().lower()
        if key not in self._instances:
            self._instances[key] = self._registry.create(key)
        return self._instances[key]

    def switch(self, name: str) -> BackendT:
        backend = self.get(name)
        self._active_name = name.strip().lower()
        return backend

    def active(self) -> Optional[BackendT]:
        if self._active_name is None:
            return None
        return self._instances[self._active_name]

    @property
    def active_name(self) -> Optional[str]:
        return self._active_name

    def available(self) -> list[str]:
        return self._registry.available()
