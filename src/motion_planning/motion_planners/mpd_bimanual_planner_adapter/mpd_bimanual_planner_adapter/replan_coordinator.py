from __future__ import annotations

import threading


class LatestOnlyReplanCoordinator:
    """At most one in-flight job; newer generations supersede old results."""
    def __init__(self, planner):
        self._planner = planner
        self._generation = 0
        self._lock = threading.Lock()

    def submit(self, request):
        with self._lock:
            self._generation += 1
            generation = self._generation
        result = self._planner(request)
        with self._lock:
            return None if generation != self._generation else result

    def invalidate(self):
        with self._lock:
            self._generation += 1
