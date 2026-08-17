# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import pytest

from franka_trajectory_jtc_test.send_mpd_trajectory import (
    FR3_JOINTS,
    validate_mpd_result,
)


def _valid_result():
    positions = np.asarray(
        [
            [0.0, -0.7, 0.0, -2.2, 0.0, 1.5, 0.7],
            [0.1, -0.6, 0.0, -2.0, 0.0, 1.6, 0.6],
            [0.2, -0.5, 0.1, -1.8, 0.2, 1.7, 0.5],
        ],
        dtype=np.float64,
    )
    return {
        "positions": positions,
        "velocities": np.zeros_like(positions),
        "accelerations": np.zeros_like(positions),
        "time_from_start": np.asarray([0.0, 1.0, 2.0]),
        "joint_names": list(FR3_JOINTS),
        "terminal_cartesian_pose_xyzw": np.asarray([0.4, 0.1, 0.6, 0.0, 0.0, 0.0, 1.0]),
    }


def test_validate_accepts_fr3_contract():
    result = _valid_result()
    validated = validate_mpd_result(
        result,
        expected_joint_names=FR3_JOINTS,
        q_start=result["positions"][0],
    )
    assert validated["positions"].shape == (3, 7)
    assert validated["terminal_cartesian_pose_xyzw"].shape == (7,)
    assert validated["joint_names"] == FR3_JOINTS


def test_validate_rejects_panda_names():
    result = _valid_result()
    result["joint_names"] = [f"panda_joint{i}" for i in range(1, 8)]
    with pytest.raises(ValueError, match="FR3"):
        validate_mpd_result(
            result,
            expected_joint_names=FR3_JOINTS,
            q_start=result["positions"][0],
        )


def test_validate_rejects_non_monotonic_time():
    result = _valid_result()
    result["time_from_start"] = np.asarray([0.0, 1.0, 1.0])
    with pytest.raises(ValueError, match="strictly increasing"):
        validate_mpd_result(
            result,
            expected_joint_names=FR3_JOINTS,
            q_start=result["positions"][0],
        )
