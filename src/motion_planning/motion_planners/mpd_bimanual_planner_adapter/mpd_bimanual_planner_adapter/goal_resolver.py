from __future__ import annotations

from .contract import JOINT_NAMES


def joint_state_by_order(names, positions):
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError("joint state names and positions must be unique and same length")
    mapping = dict(zip(names, positions))
    missing = [name for name in JOINT_NAMES if name not in mapping]
    if missing:
        raise ValueError(f"joint state is missing {missing}")
    return [float(mapping[name]) for name in JOINT_NAMES]
