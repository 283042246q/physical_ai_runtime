from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class PlanGeneration:
    generation: int
    request_seq: int
    world_version: int
    deadline_unix_ns: int


class LatestOnlyReplanCoordinator:
    """At most one in-flight job; newer generations supersede old results."""
    def __init__(self, planner):
        self._planner = planner
        self._generation = 0
        self._lock = threading.Lock()

    @property
    def generation(self):
        with self._lock:
            return self._generation

    def begin(self, *, request_seq, world_version, deadline_unix_ns):
        with self._lock:
            self._generation += 1
            return PlanGeneration(
                self._generation,
                int(request_seq),
                int(world_version),
                int(deadline_unix_ns),
            )

    def accepts(self, ticket, *, latest_world_version, now_unix_ns=None):
        now = time.time_ns() if now_unix_ns is None else int(now_unix_ns)
        with self._lock:
            return bool(
                ticket.generation == self._generation
                and ticket.world_version == int(latest_world_version)
                and now < ticket.deadline_unix_ns
            )

    def submit(self, request):
        ticket = self.begin(
            request_seq=getattr(request, "request_seq", 0),
            world_version=getattr(request, "world_version", 0),
            deadline_unix_ns=getattr(request, "deadline_unix_ns", 2**63 - 1),
        )
        result = self._planner(request)
        return result if self.accepts(
            ticket,
            latest_world_version=ticket.world_version,
        ) else None

    def invalidate(self):
        with self._lock:
            self._generation += 1
