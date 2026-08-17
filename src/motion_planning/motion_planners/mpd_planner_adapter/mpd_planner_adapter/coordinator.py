"""Bounded latest-only planning coordinator, independent of ROS."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Callable, Generic, TypeVar


T = TypeVar("T")
R = TypeVar("R")


@dataclass(frozen=True)
class Completed(Generic[T, R]):
    generation: int
    job: T
    result: R | None
    error: BaseException | None
    elapsed_s: float
    superseded: bool


class LatestOnlyPlanner(Generic[T, R]):
    """One active call plus one replaceable pending slot.

    There is no executor-owned hidden queue.  A result is marked superseded if
    a newer generation was submitted while it ran, so the ROS side can discard
    it before command publication.
    """

    def __init__(self, planner: Callable[[T], R]) -> None:
        self._planner = planner
        self._condition = threading.Condition()
        self._pending: tuple[int, T] | None = None
        self._completed: deque[Completed[T, R]] = deque(maxlen=2)
        self._latest_generation = -1
        self._active = False
        self._stopping = False
        self._thread = threading.Thread(target=self._run, name="mpd-latest-only", daemon=True)
        self._thread.start()

    @property
    def active(self) -> bool:
        with self._condition:
            return self._active

    @property
    def pending_count(self) -> int:
        with self._condition:
            return int(self._pending is not None)

    def submit(self, generation: int, job: T) -> None:
        with self._condition:
            if self._stopping:
                raise RuntimeError("planner coordinator is stopping")
            if generation <= self._latest_generation:
                raise ValueError("generation must increase monotonically")
            self._latest_generation = generation
            self._pending = (generation, job)
            self._condition.notify()

    def invalidate(self, generation: int) -> None:
        """Supersede active/pending work without starting another request."""
        with self._condition:
            if generation <= self._latest_generation:
                raise ValueError("generation must increase monotonically")
            self._latest_generation = generation
            self._pending = None

    def drain(self) -> list[Completed[T, R]]:
        with self._condition:
            completed = list(self._completed)
            self._completed.clear()
            return completed

    def close(self, timeout_s: float = 3.0) -> None:
        with self._condition:
            self._stopping = True
            self._pending = None
            self._condition.notify_all()
        self._thread.join(timeout=timeout_s)

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._stopping:
                    self._condition.wait()
                if self._stopping:
                    return
                generation, job = self._pending
                self._pending = None
                self._active = True
            started = time.perf_counter()
            result = None
            error = None
            try:
                result = self._planner(job)
            except BaseException as caught:  # surfaced to the ROS thread
                error = caught
            elapsed = time.perf_counter() - started
            with self._condition:
                self._active = False
                self._completed.append(
                    Completed(
                        generation=generation,
                        job=job,
                        result=result,
                        error=error,
                        elapsed_s=elapsed,
                        superseded=generation != self._latest_generation,
                    )
                )
