"""Offline tests for PyrokiJparseSetpointBackend."""

from __future__ import annotations

import pytest

pytest.importorskip("jax")
pytest.importorskip("pyroki")

from robot_descriptions.loaders.yourdfpy import load_robot_description

from manipulation_motion_planning.contracts import CurrentState, PoseTarget
from pyroki_planner_adapter.pyroki_setpoint_backend import PyrokiJparseSetpointBackend


@pytest.fixture(scope="module")
def panda_setpoint_backend() -> PyrokiJparseSetpointBackend:
    urdf = load_robot_description("panda_description")
    import pyroki as pk

    robot = pk.Robot.from_urdf(urdf)
    return PyrokiJparseSetpointBackend(robot, target_link_name="panda_hand")


def test_jparse_setpoint_reaches_fk_pose(
    panda_setpoint_backend: PyrokiJparseSetpointBackend,
) -> None:
    import jaxlie
    import numpy as np

    backend = panda_setpoint_backend
    names = backend._actuated_names
    cfg = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float64)
    if len(cfg) != len(names):
        cfg = np.zeros(len(names), dtype=np.float64)

    poses = backend._robot.forward_kinematics(cfg)
    ee = jaxlie.SE3(poses[backend._target_link_index])
    pos = tuple(float(x) for x in ee.translation())
    wxyz = tuple(float(x) for x in ee.rotation().wxyz)

    start = CurrentState(joint_names=names, positions=cfg.tolist())
    target = PoseTarget(position_xyz=pos, orientation_wxyz=wxyz)

    result = backend.plan(
        start,
        target,
        {
            "max_iterations": 50,
            "position_tolerance_m": 1e-3,
            "orientation_tolerance_rad": 1e-2,
        },
    )
    assert result.valid, result.reason
    assert result.joint_names == names
    assert result.positions is not None
    assert len(result.positions) == len(names)


def test_streaming_single_step_returns_best_effort_without_convergence(
    panda_setpoint_backend: PyrokiJparseSetpointBackend,
) -> None:
    backend = panda_setpoint_backend
    names = backend._actuated_names
    start = CurrentState(joint_names=names, positions=[0.0] * len(names))
    target = PoseTarget(
        position_xyz=(0.5, 0.0, 0.5),
        orientation_wxyz=(0.0, 0.0, 1.0, 0.0),
    )

    result = backend.plan(
        start,
        target,
        {"max_iterations": 1, "require_convergence": False},
    )
    assert result.valid, result.reason
    assert result.diagnostics.get("converged") is False
