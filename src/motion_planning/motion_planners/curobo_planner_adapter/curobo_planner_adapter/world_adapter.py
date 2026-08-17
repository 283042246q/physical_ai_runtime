"""Convert backend-neutral `World` snapshots to cuRobo scene configs."""

from __future__ import annotations

from dataclasses import dataclass

from . import _bootstrap  # noqa: F401

from curobo.scene import Capsule, Cuboid, Mesh, Scene, Sphere, VoxelGrid
from manipulation_motion_planning.contracts import (
    World,
    WorldBox,
    WorldCapsule,
    WorldMesh,
    WorldPose,
    WorldSphere,
    WorldVoxelGrid,
)


@dataclass
class CuroboWorldAdapter:
    """Convert `manipulation_motion_planning.World` to `curobo.scene.Scene`."""

    last_stamp_s: float = 0.0
    scene: Scene | None = None

    def update(self, world: World | None) -> Scene | None:
        if world is None:
            self.last_stamp_s = 0.0
            self.scene = None
            return None
        if world.point_clouds:
            raise NotImplementedError(
                "cuRobo point-cloud/Blox input needs a mapper integration; "
                "analytic primitives, meshes, and voxel grids are supported here."
            )

        scene = Scene(
            cuboid=[_box_to_cuboid(box) for box in world.boxes] or None,
            sphere=[_sphere_to_sphere(sphere) for sphere in world.spheres] or None,
            capsule=[_capsule_to_capsule(capsule) for capsule in world.capsules] or None,
            mesh=[_mesh_to_mesh(mesh) for mesh in world.meshes] or None,
            voxel=[_voxel_to_voxel_grid(voxel) for voxel in world.voxel_grids] or None,
        )
        self.last_stamp_s = float(world.stamp_s)
        self.scene = scene
        return scene


def _pose_to_list(pose: WorldPose) -> list[float]:
    return [
        float(pose.position_xyz[0]),
        float(pose.position_xyz[1]),
        float(pose.position_xyz[2]),
        float(pose.orientation_wxyz[0]),
        float(pose.orientation_wxyz[1]),
        float(pose.orientation_wxyz[2]),
        float(pose.orientation_wxyz[3]),
    ]


def _box_to_cuboid(box: WorldBox) -> Cuboid:
    return Cuboid(
        name=box.name,
        pose=_pose_to_list(box.pose),
        dims=[float(v) for v in box.size_xyz],
    )


def _sphere_to_sphere(sphere: WorldSphere) -> Sphere:
    return Sphere(
        name=sphere.name,
        pose=_pose_to_list(sphere.pose),
        radius=float(sphere.radius_m),
    )


def _capsule_to_capsule(capsule: WorldCapsule) -> Capsule:
    half = float(capsule.length_m) * 0.5
    return Capsule(
        name=capsule.name,
        pose=_pose_to_list(capsule.pose),
        radius=float(capsule.radius_m),
        base=[0.0, 0.0, -half],
        tip=[0.0, 0.0, half],
    )


def _mesh_to_mesh(mesh: WorldMesh) -> Mesh:
    return Mesh(
        name=mesh.name,
        pose=_pose_to_list(mesh.pose),
        file_path=mesh.mesh_uri,
        scale=[float(v) for v in mesh.scale_xyz],
    )


def _voxel_to_voxel_grid(voxel: WorldVoxelGrid) -> VoxelGrid:
    return VoxelGrid(
        name=voxel.name,
        pose=_pose_to_list(voxel.pose),
        dims=[float(v) * float(voxel.voxel_size_m) for v in voxel.dims_xyz],
        voxel_size=float(voxel.voxel_size_m),
    )
