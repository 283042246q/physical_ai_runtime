#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
#
# Pose-space normalization utilities for the execution manager.
# EM responsibility: delta→absolute.  No chunking, no IK, no FK.

import math
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped


def pose_to_array(p: Pose) -> np.ndarray:
    return np.array([p.position.x, p.position.y, p.position.z,
                     p.orientation.x, p.orientation.y,
                     p.orientation.z, p.orientation.w])


def array_to_pose(a: np.ndarray) -> Pose:
    p = Pose()
    p.position.x = float(a[0])
    p.position.y = float(a[1])
    p.position.z = float(a[2])
    p.orientation.x = float(a[3])
    p.orientation.y = float(a[4])
    p.orientation.z = float(a[5])
    p.orientation.w = float(a[6])
    return p


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """q1 * q2 (Hamilton product).  Both [x, y, z, w]."""
    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def normalize_quat(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def apply_delta(ref_pose: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """Apply a pose delta to a reference absolute pose.

    ref_pose: [x, y, z, qx, qy, qz, qw] — absolute world-frame pose
    delta:    [dx, dy, dz, dqx, dqy, dqz, dqw] — delta in world frame

    Position: ref + delta
    Orientation: delta_quat * ref_quat
    """
    result = np.empty(7)
    result[0:3] = ref_pose[0:3] + delta[0:3]
    dq = normalize_quat(delta[3:7])
    rq = normalize_quat(ref_pose[3:7])
    result[3:7] = quat_multiply(dq, rq)
    return result


def pose_stamped_to_array(msg: PoseStamped) -> np.ndarray:
    return pose_to_array(msg.pose)


def is_valid_absolute_pose(msg: PoseStamped) -> bool:
    """Check that all pose components are finite."""
    p = msg.pose
    return all(math.isfinite(v) for v in [
        p.position.x, p.position.y, p.position.z,
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
