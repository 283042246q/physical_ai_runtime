from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WorldSnapshot:
    world_version: int
    valid_until_ns: int
    objects: tuple


class LatestWorldBuffer:
    def __init__(self):
        self._snapshot = None

    @property
    def snapshot(self):
        return self._snapshot

    def update(self, message):
        version = int(message.world_version)
        if self._snapshot is not None and version <= self._snapshot.world_version:
            raise ValueError("world_version must increase monotonically")
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        valid = message.valid_until.sec * 1_000_000_000 + message.valid_until.nanosec
        if valid <= stamp:
            raise ValueError("DynamicWorld.valid_until must be after its header stamp")
        self._snapshot = WorldSnapshot(version, valid, tuple(message.objects))
        return self._snapshot
