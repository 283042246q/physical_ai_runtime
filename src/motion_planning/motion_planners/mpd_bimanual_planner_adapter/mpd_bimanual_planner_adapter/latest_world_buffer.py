from __future__ import annotations

from dataclasses import dataclass
import math
import threading


@dataclass(frozen=True)
class WorldSnapshot:
    world_version: int
    frame_id: str
    stamp_unix_ns: int
    valid_until_ns: int
    objects: tuple

    def to_worker_payload(self):
        return {
            "world_version": self.world_version,
            "frame_id": self.frame_id,
            "stamp_unix_ns": self.stamp_unix_ns,
            "valid_until_unix_ns": self.valid_until_ns,
            "objects": [dict(item) for item in self.objects],
        }


class LatestWorldBuffer:
    def __init__(self, *, planning_frame="world", max_objects=16):
        self._snapshot = None
        self.planning_frame = str(planning_frame)
        self.max_objects = int(max_objects)
        self._lock = threading.Lock()

    @property
    def snapshot(self):
        with self._lock:
            return self._snapshot

    @staticmethod
    def _finite(values, size, field):
        if len(values) != size:
            raise ValueError(f"{field} must contain {size} values")
        result = [float(value) for value in values]
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{field} contains NaN or Inf")
        return result

    def _object(self, item, world_valid_until_ns):
        object_id = str(item.object_id)
        if not object_id:
            raise ValueError("dynamic object_id must be non-empty")
        sizes = self._finite(item.size_xyz, 3, f"{object_id}.size_xyz")
        if any(value <= 0.0 for value in sizes):
            raise ValueError(f"{object_id}.size_xyz must be positive")
        if item.shape_type == item.SPHERE:
            local_sdf = {"type": "sphere", "radius": sizes[0]}
        elif item.shape_type == item.BOX:
            local_sdf = {"type": "box", "size_xyz": sizes}
        elif item.shape_type == item.CAPSULE:
            local_sdf = {"type": "capsule", "radius": sizes[0], "length": sizes[2]}
        else:
            raise ValueError(f"{object_id} has unsupported shape_type")
        pose = item.pose
        position = self._finite(
            [pose.position.x, pose.position.y, pose.position.z], 3, f"{object_id}.position"
        )
        orientation = self._finite(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            4,
            f"{object_id}.orientation",
        )
        if math.sqrt(sum(value * value for value in orientation)) < 1e-9:
            raise ValueError(f"{object_id}.orientation quaternion is zero")
        twist = item.twist
        velocity = self._finite(
            [twist.linear.x, twist.linear.y, twist.linear.z], 3, f"{object_id}.velocity"
        )
        covariance = self._finite(item.covariance, 36, f"{object_id}.covariance")
        for row in range(6):
            if covariance[row * 6 + row] < 0.0:
                raise ValueError(f"{object_id}.covariance diagonal must be non-negative")
            for column in range(6):
                if abs(covariance[row * 6 + column] - covariance[column * 6 + row]) > 1e-9:
                    raise ValueError(f"{object_id}.covariance must be symmetric")
        inflation = float(item.inflation_m)
        if not math.isfinite(inflation) or inflation < 0.0:
            raise ValueError(f"{object_id}.inflation_m must be finite and non-negative")
        object_valid = int(item.valid_until.sec) * 1_000_000_000 + int(item.valid_until.nanosec)
        if object_valid <= 0 or object_valid < world_valid_until_ns:
            raise ValueError(f"{object_id}.valid_until must cover world.valid_until")
        return {
            "id": object_id,
            "local_sdf": local_sdf,
            "pose": {"position": position, "orientation_xyzw": orientation},
            "linear_velocity": velocity,
            "covariance_6x6": covariance,
            "inflation": {"mode": "covariance", "base_m": inflation},
            "valid_until_unix_ns": object_valid,
        }

    def update(self, message):
        version = int(message.world_version)
        if message.header.frame_id != self.planning_frame:
            raise ValueError(
                f"DynamicWorld frame must be {self.planning_frame!r}, got {message.header.frame_id!r}"
            )
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        valid = message.valid_until.sec * 1_000_000_000 + message.valid_until.nanosec
        if stamp <= 0:
            raise ValueError("DynamicWorld header stamp must be non-zero")
        if valid <= stamp:
            raise ValueError("DynamicWorld.valid_until must be after its header stamp")
        if len(message.objects) > self.max_objects:
            raise ValueError(f"DynamicWorld exceeds fixed capacity {self.max_objects}")
        objects = tuple(self._object(item, valid) for item in message.objects)
        ids = [item["id"] for item in objects]
        if len(ids) != len(set(ids)):
            raise ValueError("DynamicWorld object ids must be unique")
        with self._lock:
            if self._snapshot is not None and version <= self._snapshot.world_version:
                raise ValueError("world_version must increase monotonically")
            self._snapshot = WorldSnapshot(
                version, self.planning_frame, int(stamp), int(valid), objects
            )
            return self._snapshot
