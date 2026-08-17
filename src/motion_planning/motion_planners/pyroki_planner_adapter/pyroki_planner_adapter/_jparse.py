"""J-PARSE velocity IK step used by `PyrokiJparseSetpointBackend`.

PyRoki publishes its reusable library as `pyroki`; the example-only
`pyroki_snippets` package is not part of the installed distribution. This file
keeps the small J-PARSE step we depend on inside the adapter package.
"""

from __future__ import annotations

from typing import Literal

import jax
import jax.numpy as jnp
import jaxlie
import numpy as np
import pyroki as pk
from jax.typing import ArrayLike


def compute_jacobian(
    robot: pk.Robot,
    cfg: ArrayLike,
    target_link_index: int,
    position_only: bool = True,
) -> jnp.ndarray:
    """Compute geometric Jacobian via autodiff on PyRoki FK."""

    cfg = jnp.asarray(cfg)

    if position_only:
        jacobian = jax.jacfwd(
            lambda q: jaxlie.SE3(robot.forward_kinematics(q)).translation()
        )(cfg)[target_link_index]
    else:
        anchor_poses = robot.forward_kinematics(cfg)
        r_anchor_inv = jaxlie.SE3(anchor_poses[target_link_index]).rotation().inverse()

        def get_pose_components(q: jax.Array) -> jnp.ndarray:
            poses = robot.forward_kinematics(q)
            pose = jaxlie.SE3(poses[target_link_index])
            relative_rotation = pose.rotation() @ r_anchor_inv
            return jnp.concatenate([pose.translation(), relative_rotation.log()])

        jacobian = jax.jacfwd(get_pose_components)(cfg)

    return jacobian


def jparse_pseudoinverse(
    jacobian: ArrayLike,
    gamma: float = 0.1,
    singular_direction_gain_position: float = 1.0,
    singular_direction_gain_angular: float = 1.0,
    position_dimensions: int | None = None,
    angular_dimensions: int | None = None,
) -> tuple[jnp.ndarray, jnp.ndarray]:
    """Compute J-PARSE pseudo-inverse and nullspace projector."""

    j_mat = jnp.asarray(jacobian)
    m, n = j_mat.shape

    if position_dimensions is None and angular_dimensions is None:
        pos_dims = m
        ang_dims = 0
    else:
        if position_dimensions is None or angular_dimensions is None:
            raise ValueError(
                "Both position_dimensions and angular_dimensions must be provided."
            )
        if position_dimensions + angular_dimensions != m:
            raise ValueError(
                "position_dimensions + angular_dimensions must equal Jacobian row count."
            )
        pos_dims = position_dimensions
        ang_dims = angular_dimensions

    u_mat, singular_values, vt_mat = jnp.linalg.svd(j_mat, full_matrices=True)
    k_count = singular_values.shape[0]

    sigma_max = jnp.max(singular_values)
    threshold = gamma * sigma_max
    non_singular = singular_values > threshold

    safety_singular_values = jnp.where(non_singular, singular_values, threshold)
    projected_singular_values = jnp.where(non_singular, singular_values, 0.0)

    u_k = u_mat[:, :k_count]
    vt_k = vt_mat[:k_count, :]

    safety_jacobian = u_k * safety_singular_values[None, :] @ vt_k
    projected_jacobian = u_k * projected_singular_values[None, :] @ vt_k

    safety_pinv = jnp.linalg.pinv(safety_jacobian)
    projected_pinv = jnp.linalg.pinv(projected_jacobian)

    phi = jnp.where(non_singular, 0.0, singular_values / (sigma_max * gamma))
    singular_gains = jnp.concatenate(
        [
            jnp.full((pos_dims,), singular_direction_gain_position),
            jnp.full((ang_dims,), singular_direction_gain_angular),
        ]
    )
    gain_matrix = jnp.diag(singular_gains)
    singular_feedback = (u_k * phi[None, :]) @ u_k.T @ gain_matrix

    jparse_inv = (
        safety_pinv @ projected_jacobian @ projected_pinv
        + safety_pinv @ singular_feedback
    )
    nullspace = jnp.eye(n) - safety_pinv @ safety_jacobian

    return jparse_inv, nullspace


def pinv(jacobian: ArrayLike) -> jnp.ndarray:
    """Standard Moore-Penrose pseudo-inverse."""

    return jnp.linalg.pinv(jnp.asarray(jacobian))


def damped_least_squares(jacobian: ArrayLike, damping: float = 0.05) -> jnp.ndarray:
    """Damped least squares pseudo-inverse."""

    j_mat = jnp.asarray(jacobian)
    n_cols = j_mat.shape[1]
    return jnp.linalg.inv(j_mat.T @ j_mat + damping**2 * jnp.eye(n_cols)) @ j_mat.T


def manipulability_measure(jacobian: ArrayLike) -> jnp.ndarray:
    """Yoshikawa manipulability measure."""

    j_mat = jnp.asarray(jacobian)
    return jnp.sqrt(jnp.linalg.det(j_mat @ j_mat.T))


def inverse_condition_number(jacobian: ArrayLike) -> jnp.ndarray:
    """Inverse condition number: sigma_min / sigma_max."""

    singular_values = jnp.linalg.svd(jnp.asarray(jacobian), compute_uv=False)
    return jnp.min(singular_values) / jnp.max(singular_values)


def jparse_step(
    robot: pk.Robot,
    cfg: ArrayLike,
    target_link_index: int,
    target_position: ArrayLike,
    target_wxyz: ArrayLike | None = None,
    *,
    method: Literal["jparse", "pinv", "dls"] = "jparse",
    gamma: float = 0.1,
    singular_direction_gain_position: float = 1.0,
    singular_direction_gain_angular: float = 1.0,
    position_gain: float = 5.0,
    orientation_gain: float = 1.0,
    nullspace_gain: float = 0.5,
    max_joint_velocity: float = 2.0,
    dls_damping: float = 0.05,
    dt: float = 0.02,
    home_cfg: ArrayLike | None = None,
) -> tuple[np.ndarray, dict]:
    """Run one J-PARSE, pseudo-inverse, or DLS velocity IK step."""

    cfg = jnp.asarray(cfg)
    target_position = jnp.asarray(target_position)
    position_only = target_wxyz is None

    poses = robot.forward_kinematics(cfg)
    current_pose = jaxlie.SE3(poses[target_link_index])
    current_pos = current_pose.translation()

    pos_error = target_position - current_pos
    pos_error_mag = float(jnp.linalg.norm(pos_error))

    omega_error = jnp.zeros(3)
    if position_only:
        v_des = position_gain * pos_error
    else:
        assert target_wxyz is not None
        target_quat = jnp.asarray(target_wxyz)
        target_quat = target_quat / jnp.linalg.norm(target_quat)

        current_quat = current_pose.rotation().wxyz
        current_quat = current_quat / jnp.linalg.norm(current_quat)

        target_quat = jnp.asarray(
            jnp.where(jnp.dot(target_quat, current_quat) < 0, -target_quat, target_quat)
        )

        q_current = jaxlie.SO3(current_quat)
        q_target = jaxlie.SO3(target_quat)
        omega_error = (q_target @ q_current.inverse()).log()

        omega_mag = jnp.linalg.norm(omega_error)
        max_omega = 1.0
        omega_error = jnp.asarray(
            jnp.where(
                omega_mag > max_omega,
                omega_error * max_omega / omega_mag,
                omega_error,
            )
        )

        v_des = jnp.concatenate(
            [position_gain * pos_error, orientation_gain * omega_error]
        )

    jacobian = compute_jacobian(
        robot,
        cfg,
        target_link_index,
        position_only=position_only,
    )

    if method == "jparse":
        j_inv, nullspace = jparse_pseudoinverse(
            jacobian,
            gamma=gamma,
            singular_direction_gain_position=singular_direction_gain_position,
            singular_direction_gain_angular=singular_direction_gain_angular,
            position_dimensions=3,
            angular_dimensions=0 if position_only else 3,
        )
    elif method == "pinv":
        j_inv = pinv(jacobian)
        nullspace = jnp.eye(jacobian.shape[1]) - j_inv @ jacobian
    else:
        j_inv = damped_least_squares(jacobian, dls_damping)
        nullspace = jnp.eye(jacobian.shape[1]) - j_inv @ jacobian

    dq = j_inv @ v_des

    if nullspace_gain > 0:
        if home_cfg is None:
            lower = robot.joints.lower_limits
            upper = robot.joints.upper_limits
            home = (lower + upper) / 2.0
        else:
            home = jnp.asarray(home_cfg)
        dq = dq + nullspace @ (-nullspace_gain * (cfg - home))

    max_joint_vel = float(jnp.max(jnp.abs(dq)))
    scale = jnp.where(
        jnp.max(jnp.abs(dq)) > max_joint_velocity,
        max_joint_velocity / jnp.max(jnp.abs(dq)),
        1.0,
    )
    dq = dq * scale

    new_cfg = cfg + dq * dt
    new_cfg = jnp.clip(new_cfg, robot.joints.lower_limits, robot.joints.upper_limits)

    info = {
        "position_error": pos_error_mag,
        "orientation_error": float(jnp.linalg.norm(omega_error))
        if not position_only
        else 0.0,
        "max_joint_vel": max_joint_vel,
        "jacobian": np.asarray(jacobian),
        "manipulability": float(manipulability_measure(jacobian)),
        "inverse_condition_number": float(inverse_condition_number(jacobian)),
    }

    return np.asarray(new_cfg), info
