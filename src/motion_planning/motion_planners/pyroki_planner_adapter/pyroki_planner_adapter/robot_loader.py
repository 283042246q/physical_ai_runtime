"""Load PyRoki robot and collision models from URDF."""

from __future__ import annotations

import io
from pathlib import Path
from typing import Union

from . import _bootstrap  # noqa: F401

import pyroki as pk
import yourdfpy

UrdfSource = Union[str, Path]


def _is_urdf_xml(text: str) -> bool:
    stripped = text.lstrip()
    return stripped.startswith("<?xml") or stripped.startswith("<robot")


def load_urdf_model(urdf: UrdfSource, *, load_meshes: bool = False) -> yourdfpy.URDF:
    """Load a `yourdfpy.URDF` from a path or inline XML string."""

    if isinstance(urdf, Path):
        return yourdfpy.URDF.load(str(urdf), load_meshes=load_meshes)
    if isinstance(urdf, str) and _is_urdf_xml(urdf):
        return yourdfpy.URDF.load(
            io.BytesIO(urdf.encode("utf-8")),
            load_meshes=load_meshes,
        )

    urdf_path = Path(urdf)
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF path does not exist: {urdf}")
    return yourdfpy.URDF.load(str(urdf_path), load_meshes=load_meshes)


def load_robot_from_urdf(urdf: UrdfSource, *, load_meshes: bool = False) -> pk.Robot:
    """Build a `pk.Robot` from a URDF path or inline XML string."""

    model = load_urdf_model(urdf, load_meshes=load_meshes)
    return pk.Robot.from_urdf(model)


def load_robot_collision_from_urdf(
    urdf: UrdfSource,
    *,
    load_meshes: bool = True,
) -> pk.collision.RobotCollision:
    """Build PyRoki robot collision geometry from URDF collision meshes."""

    model = load_urdf_model(urdf, load_meshes=load_meshes)
    return pk.collision.RobotCollision.from_urdf(model)
