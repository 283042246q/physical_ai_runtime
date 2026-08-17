"""Convert backend-neutral `World` snapshots to PyRoki collision geometry."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from . import _bootstrap  # noqa: F401

import pyroki as pk
from manipulation_motion_planning.contracts import (
    World,
    WorldBox,
    WorldCapsule,
    WorldSphere,
)


@dataclass
class PyrokiWorldAdapter:
    """Convert `manipulation_motion_planning.World` to PyRoki collisions."""

    last_stamp_s: float = 0.0
    obstacles: list[pk.collision.CollGeom] = field(default_factory=list)

    def update(self, world: World | None) -> list[pk.collision.CollGeom]:
        if world is None:
            self.last_stamp_s = 0.0
            self.obstacles = []
            return []

        self._reject_unsupported_dense_input(world)
        obstacles: list[pk.collision.CollGeom] = []
        obstacles.extend(_boxes_to_pyroki(world.boxes))
        obstacles.extend(_spheres_to_pyroki(world.spheres))
        obstacles.extend(_capsules_to_pyroki(world.capsules))
        self.last_stamp_s = float(world.stamp_s)
        self.obstacles = obstacles
        return list(obstacles)

    @staticmethod
    def _reject_unsupported_dense_input(world: World) -> None:
        unsupported = []
        if world.meshes:
            unsupported.append("meshes")
        if world.voxel_grids:
            unsupported.append("voxel_grids")
        if world.point_clouds:
            unsupported.append("point_clouds")
        if unsupported:
            raise NotImplementedError(
                "PyRoki world adapter currently supports analytic primitives only; "
                f"unsupported fields present: {unsupported}"
            )


def _boxes_to_pyroki(boxes: Sequence[WorldBox]) -> list[pk.collision.Box]:
    return [
        pk.collision.Box.from_extent(
            extent=box.size_xyz,
            position=box.pose.position_xyz,
            wxyz=box.pose.orientation_wxyz,
        )
        for box in boxes
    ]


def _spheres_to_pyroki(spheres: Sequence[WorldSphere]) -> list[pk.collision.Sphere]:
    return [
        pk.collision.Sphere.from_center_and_radius(
            center=sphere.pose.position_xyz,
            radius=sphere.radius_m,
        )
        for sphere in spheres
    ]


def _capsules_to_pyroki(capsules: Sequence[WorldCapsule]) -> list[pk.collision.Capsule]:
    return [
        pk.collision.Capsule.from_radius_height(
            radius=capsule.radius_m,
            height=capsule.length_m,
            position=capsule.pose.position_xyz,
            wxyz=capsule.pose.orientation_wxyz,
        )
        for capsule in capsules
    ]
