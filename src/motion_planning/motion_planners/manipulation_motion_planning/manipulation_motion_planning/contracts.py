"""Backend-neutral data shapes for motion-planner sources.

Three planner families (docs/MOTION_PLANNER_SOURCE_INTERFACE.md Section 5):

| Family | Backend | Result / point types | EM contract |
|---|---|---|---|
| Global setpoint | `GlobalSetpointBackend` | `SetpointPlanResult` | `joint_target` → JSPC |
| Global trajectory | `GlobalTrajectoryBackend` | `TrajectoryPlanPoint`, `TrajectoryPlanResult` | `joint_trajectory_goal` → JTC |
| Online horizon MPC | `OnlineMpcBackend` | `HorizonPlanPoint`, `HorizonPlanResult` | `joint_chunk` → JSPC |
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class CurrentState:
    """Latest known joint state, name-matched and freshness-checked.

    Used as the online MPC per-tick state and as the global planner start
    state (`StartState` alias).
    """

    joint_names: list[str]
    positions: list[float]
    velocities: Optional[list[float]] = None
    stamp_s: float = 0.0


StartState = CurrentState


@dataclass(frozen=True)
class PoseTarget:
    """Cartesian target in the planner's configured base frame."""

    position_xyz: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]
    stamp_s: float = 0.0


@dataclass(frozen=True)
class JointTarget:
    """Joint-space goal passed directly to a backend."""

    joint_names: list[str]
    positions: list[float]
    stamp_s: float = 0.0


Target = PoseTarget | JointTarget


@dataclass(frozen=True)
class WorldPose:
    """Pose in the planner world frame: xyz + wxyz quaternion."""

    position_xyz: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class WorldBox:
    """Collision box with dimensions in meters."""

    name: str
    pose: WorldPose
    size_xyz: tuple[float, float, float]


@dataclass(frozen=True)
class WorldSphere:
    """Collision sphere with radius in meters."""

    name: str
    pose: WorldPose
    radius_m: float


@dataclass(frozen=True)
class WorldCapsule:
    """Collision capsule with axis along local z."""

    name: str
    pose: WorldPose
    radius_m: float
    length_m: float


@dataclass(frozen=True)
class WorldMesh:
    """Collision mesh reference. Vertices are intentionally not embedded here."""

    name: str
    pose: WorldPose
    mesh_uri: str
    scale_xyz: tuple[float, float, float] = (1.0, 1.0, 1.0)


@dataclass(frozen=True)
class WorldVoxelGrid:
    """Voxel occupancy grid in the planner world frame."""

    name: str
    pose: WorldPose
    voxel_size_m: float
    dims_xyz: tuple[int, int, int]
    occupied_indices: list[tuple[int, int, int]] = field(default_factory=list)


@dataclass(frozen=True)
class WorldPointCloud:
    """Raw or filtered point cloud snapshot in the planner world frame."""

    name: str
    frame_id: str
    points_xyz: list[tuple[float, float, float]] = field(default_factory=list)


@dataclass(frozen=True)
class World:
    """Backend-neutral collision/planning-scene snapshot.

    This is the boundary between perception/environment sources and planner
    backends. PyRoki adapters convert these objects to `pk.collision.CollGeom`;
    cuRobo adapters convert them to `WorldConfig`, `VoxelGrid`, or Blox input.
    """

    stamp_s: float = 0.0
    frame_id: str = "world"
    boxes: list[WorldBox] = field(default_factory=list)
    spheres: list[WorldSphere] = field(default_factory=list)
    capsules: list[WorldCapsule] = field(default_factory=list)
    meshes: list[WorldMesh] = field(default_factory=list)
    voxel_grids: list[WorldVoxelGrid] = field(default_factory=list)
    point_clouds: list[WorldPointCloud] = field(default_factory=list)


# -- Global trajectory (Section 6.1.2) ------------------------------------


@dataclass(frozen=True)
class TrajectoryPlanPoint:
    """One waypoint of a `GlobalTrajectoryBackend` result."""

    positions: list[float]
    velocities: Optional[list[float]] = None
    time_from_start_s: float = 0.0


@dataclass
class TrajectoryPlanResult:
    """Result of `GlobalTrajectoryBackend.plan()` → EM `joint_trajectory_goal`."""

    valid: bool
    joint_names: Optional[list[str]] = None
    points: list[TrajectoryPlanPoint] = field(default_factory=list)
    reason: Optional[str] = None
    diagnostics: dict = field(default_factory=dict)


# -- Global setpoint (Section 6.1.1) --------------------------------------


@dataclass
class SetpointPlanResult:
    """Result of `GlobalSetpointBackend.plan()` → EM `joint_target`."""

    valid: bool
    joint_names: Optional[list[str]] = None
    positions: Optional[list[float]] = None
    reason: Optional[str] = None
    diagnostics: dict = field(default_factory=dict)


# -- Online horizon MPC (Section 6.2) -------------------------------------


@dataclass(frozen=True)
class HorizonPlanPoint:
    """One joint-space sample on the receding horizon from an MPC tick.

    Maps to one `trajectory_msgs/JointTrajectoryPoint` in the published
    `joint_chunk`. `time_from_start_s == 0.0` is the immediate command.
    """

    positions: list[float]
    velocities: Optional[list[float]] = None
    time_from_start_s: float = 0.0


@dataclass
class HorizonPlanResult:
    """Result of `OnlineMpcBackend.step()` → EM `joint_chunk`."""

    valid: bool
    points: list[HorizonPlanPoint] = field(default_factory=list)
    reason: Optional[str] = None
    diagnostics: dict = field(default_factory=dict)
