#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Plan one FR3 trajectory with MPD and optionally execute it through EM.

The ROS-facing request, saved result, and submitted trajectory use FR3 names.
The safe default is ``plan_only:=true``: inference and validation run, but no
robot command is submitted. The current checkpoint continues to use its
original Panda-trained planning backend internally.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


FR3_JOINTS = [f"fr3_joint{index}" for index in range(1, 8)]
GOAL_TYPES = ("cartesian", "joint")
DEFAULT_Q_GOAL = [0.2, -0.3, 0.1, -1.8, 0.2, 1.6, 0.1]
DEFAULT_EE_POSE_GOAL = [
    0.4322542996381046,
    0.16375043690143717,
    0.6717085498613047,
    0.8765521159636589,
    0.47117624388215046,
    0.06455630619812767,
    -0.07403930395941108,
]
DEFAULT_CONDA = Path("/home/eric/anaconda3/bin/conda")
DEFAULT_CONDA_PREFIX = Path("/home/eric/anaconda3/envs/mpd-splines-public")
DEFAULT_INFER_ONCE = Path(
    "/home/eric/Projects/MotionPlanningDiffusion/mpd/scripts/runtime/infer_once.py"
)


def validate_mpd_result(
    result: dict,
    *,
    expected_joint_names: list[str],
    q_start: np.ndarray,
) -> dict:
    """Normalize and validate the neutral MPD trajectory contract."""
    normalized = {
        "positions": np.asarray(result["positions"], dtype=np.float64),
        "velocities": np.asarray(result["velocities"], dtype=np.float64),
        "accelerations": np.asarray(result["accelerations"], dtype=np.float64),
        "time_from_start": np.asarray(result["time_from_start"], dtype=np.float64),
        "joint_names": list(result["joint_names"]),
        "terminal_cartesian_pose_xyzw": np.asarray(
            result["terminal_cartesian_pose_xyzw"], dtype=np.float64
        ),
    }
    positions = normalized["positions"]
    velocities = normalized["velocities"]
    accelerations = normalized["accelerations"]
    times = normalized["time_from_start"]
    terminal_cartesian_pose = normalized["terminal_cartesian_pose_xyzw"]

    if normalized["joint_names"] != expected_joint_names:
        raise ValueError("MPD result joint_names do not match the ordered FR3 joints")
    if positions.ndim != 2 or positions.shape[1] != len(expected_joint_names):
        raise ValueError("positions must have shape [T, 7]")
    if positions.shape[0] < 2:
        raise ValueError("trajectory must contain at least two points")
    if velocities.shape != positions.shape:
        raise ValueError("velocities must match positions shape")
    if accelerations.shape != positions.shape:
        raise ValueError("accelerations must match positions shape")
    if times.shape != (positions.shape[0],):
        raise ValueError("time_from_start must have shape [T]")
    if terminal_cartesian_pose.shape != (7,):
        raise ValueError("terminal_cartesian_pose_xyzw must have shape [7]")
    if not all(
        np.isfinite(values).all()
        for values in (
            positions,
            velocities,
            accelerations,
            times,
            terminal_cartesian_pose,
        )
    ):
        raise ValueError("trajectory contains NaN or Inf")
    quaternion_norm = np.linalg.norm(terminal_cartesian_pose[3:])
    if not np.isclose(quaternion_norm, 1.0, atol=1e-4):
        raise ValueError("cartesian trajectory contains non-unit quaternions")
    if abs(float(times[0])) > 1e-8:
        raise ValueError("time_from_start must begin at zero")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError("time_from_start must be strictly increasing")

    start_error = float(np.max(np.abs(positions[0] - q_start)))
    if start_error > 1e-5:
        raise ValueError(
            f"trajectory start differs from request by {start_error:.6g} rad"
        )
    return normalized


class MpdPlanner:
    """File-based adapter for the isolated MPD Conda runtime."""

    def __init__(
        self,
        *,
        conda: Path,
        conda_env: str,
        conda_prefix: Path,
        infer_once: Path,
        output_root: Path,
        device: str,
        timeout_s: float,
    ) -> None:
        self.conda = conda
        self.conda_env = conda_env
        self.conda_prefix = conda_prefix
        self.infer_once = infer_once
        self.output_root = output_root
        self.device = device
        self.timeout_s = timeout_s

    def plan(self, request: dict) -> dict:
        """Run one FR3-named MPD request and return its saved trajectory."""
        request_id = str(request["request_id"])
        output_dir = self.output_root / request_id
        output_dir.mkdir(parents=True, exist_ok=False)
        request_path = output_dir / "request.json"
        payload = {
            key: value.tolist() if isinstance(value, np.ndarray) else value
            for key, value in request.items()
        }
        request_path.write_text(
            json.dumps(payload, allow_nan=False, indent=2), encoding="utf-8"
        )

        child_env = os.environ.copy()
        child_env.pop("PYTHONPATH", None)
        child_env.pop("PYTHONHOME", None)
        conda_lib = str(self.conda_prefix / "lib")
        inherited_library_path = child_env.get("LD_LIBRARY_PATH")
        child_env["LD_LIBRARY_PATH"] = (
            f"{conda_lib}:{inherited_library_path}"
            if inherited_library_path
            else conda_lib
        )
        completed = subprocess.run(
            [
                str(self.conda),
                "run",
                "--no-capture-output",
                "-n",
                self.conda_env,
                "python",
                str(self.infer_once),
                "--request",
                str(request_path),
                "--output-dir",
                str(output_dir),
                "--device",
                self.device,
            ],
            capture_output=True,
            text=True,
            timeout=self.timeout_s,
            check=False,
            env=child_env,
        )
        result_path = output_dir / "result.json"
        result_payload = (
            json.loads(result_path.read_text(encoding="utf-8"))
            if result_path.is_file()
            else {}
        )
        if completed.returncode != 0 or result_payload.get("status") != "success":
            error = result_payload.get("error", {})
            message = error.get("message") or completed.stderr.strip()
            raise RuntimeError(message or "MPD inference failed")
        if result_payload.get("request_id") != request_id:
            raise RuntimeError("MPD result request_id does not match request")

        trajectory_path = output_dir / result_payload["trajectory_file"]
        with np.load(trajectory_path, allow_pickle=False) as trajectory:
            trajectory_result = {
                "positions": trajectory["positions"].copy(),
                "velocities": trajectory["velocities"].copy(),
                "accelerations": trajectory["accelerations"].copy(),
                "time_from_start": trajectory["time_from_start"].copy(),
                "joint_names": trajectory["joint_names"].tolist(),
                "terminal_cartesian_pose_xyzw": trajectory[
                    "terminal_cartesian_pose_xyzw"
                ].copy(),
            }
        trajectory_result["output_dir"] = output_dir
        trajectory_result["result_payload"] = result_payload
        return trajectory_result


class SendMpdTrajectory(Node):
    """One-shot FR3 MPD planner with plan-only and EM execution modes."""

    def __init__(self) -> None:
        super().__init__("send_mpd_trajectory")
        self.declare_parameter("joint_names", FR3_JOINTS)
        self.declare_parameter("joint_state_topic", "/franka/joint_states")
        self.declare_parameter(
            "trajectory_action",
            "/action_sources/trajectory_test/arm/joint_trajectory",
        )
        self.declare_parameter("robot_model", "franka_fr3")
        self.declare_parameter("planning_frame", "fr3_link0")
        self.declare_parameter("scene_id", "EnvWarehouseExtraObjectsV00")
        self.declare_parameter("goal_type", "cartesian")
        self.declare_parameter("ee_pose_goal", DEFAULT_EE_POSE_GOAL)
        self.declare_parameter("q_pos_goal", DEFAULT_Q_GOAL)
        self.declare_parameter("ik_candidates", 0)
        self.declare_parameter("ik_max_iters", 300)
        self.declare_parameter("seed", 12345)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("plan_only", True)
        self.declare_parameter("output_dir", "/tmp/mpd-fr3-plans")
        self.declare_parameter("mpd_conda", str(DEFAULT_CONDA))
        self.declare_parameter("mpd_conda_env", "mpd-splines-public")
        self.declare_parameter("mpd_conda_prefix", str(DEFAULT_CONDA_PREFIX))
        self.declare_parameter("mpd_infer_once", str(DEFAULT_INFER_ONCE))
        self.declare_parameter("planning_timeout_s", 900.0)
        self.declare_parameter("wait_for_em_s", 15.0)
        self.declare_parameter("result_timeout_margin_s", 10.0)
        self.declare_parameter("max_start_drift_rad", 0.01)
        self.declare_parameter("max_joint_state_age_s", 0.5)

        self.joint_names = list(self.get_parameter("joint_names").value)
        if self.joint_names != FR3_JOINTS:
            raise ValueError("joint_names must be fr3_joint1 through fr3_joint7")
        self.goal_type = str(self.get_parameter("goal_type").value)
        if self.goal_type not in GOAL_TYPES:
            raise ValueError(f"goal_type must be one of {list(GOAL_TYPES)}")
        self.ik_candidates = int(self.get_parameter("ik_candidates").value)
        self.ik_max_iters = int(self.get_parameter("ik_max_iters").value)
        if not 0 <= self.ik_candidates <= 256:
            raise ValueError("ik_candidates must be in [0, 256]")
        if not 1 <= self.ik_max_iters <= 2000:
            raise ValueError("ik_max_iters must be in [1, 2000]")
        self.ee_goal = np.asarray(
            self.get_parameter("ee_pose_goal").value, dtype=np.float64
        )
        self.q_goal = np.asarray(
            self.get_parameter("q_pos_goal").value, dtype=np.float64
        )
        if self.ee_goal.shape != (7,):
            raise ValueError("ee_pose_goal must have shape [7]: [x,y,z,qx,qy,qz,qw]")
        if self.q_goal.shape != (len(FR3_JOINTS),):
            raise ValueError("q_pos_goal must have shape [7]")
        if not np.isfinite(self.ee_goal).all() or not np.isfinite(self.q_goal).all():
            raise ValueError("goal contains NaN or Inf")
        quaternion_norm = float(np.linalg.norm(self.ee_goal[3:]))
        if not 0.99 <= quaternion_norm <= 1.01:
            raise ValueError(
                f"ee_pose_goal quaternion xyzw must have unit norm, got {quaternion_norm:.6f}"
            )
        self.ee_goal[3:] /= quaternion_norm

        self.plan_only = bool(self.get_parameter("plan_only").value)
        self.wait_for_em_s = float(self.get_parameter("wait_for_em_s").value)
        self.result_timeout_margin_s = float(
            self.get_parameter("result_timeout_margin_s").value
        )
        self.max_start_drift_rad = float(
            self.get_parameter("max_start_drift_rad").value
        )
        self.max_joint_state_age_s = float(
            self.get_parameter("max_joint_state_age_s").value
        )
        self.planner = MpdPlanner(
            conda=Path(str(self.get_parameter("mpd_conda").value)),
            conda_env=str(self.get_parameter("mpd_conda_env").value),
            conda_prefix=Path(str(self.get_parameter("mpd_conda_prefix").value)),
            infer_once=Path(str(self.get_parameter("mpd_infer_once").value)),
            output_root=Path(str(self.get_parameter("output_dir").value)).expanduser(),
            device=str(self.get_parameter("device").value),
            timeout_s=float(self.get_parameter("planning_timeout_s").value),
        )

        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("trajectory_action").value),
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.state: np.ndarray | None = None
        self._state_received_at: float | None = None
        self._planned_from: np.ndarray | None = None
        self._plan_future: Future | None = None
        self._plan_result: dict | None = None
        self._published_at: float | None = None
        self._goal_handle = None
        self._expected_duration_s = 0.0
        self._saw_executing = False
        self._saw_succeeded = False
        self._ready_since: float | None = None
        self._worker = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="mpd-planner"
        )
        self.exit_code = 1
        self.create_timer(0.1, self._tick)
        configured_goal = self.ee_goal if self.goal_type == "cartesian" else self.q_goal
        self.get_logger().info(
            "Waiting for FR3 joint state; "
            f"plan_only={self.plan_only}, goal_type={self.goal_type}, "
            f"goal={configured_goal.tolist()}, "
            f"ik_candidates={self.ik_candidates}, ik_max_iters={self.ik_max_iters}"
        )

    def close(self) -> None:
        self._worker.shutdown(wait=False, cancel_futures=True)

    def _on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.joint_names):
            return
        state = np.asarray(
            [positions[name] for name in self.joint_names], dtype=np.float64
        )
        if np.isfinite(state).all():
            self.state = state
            self._state_received_at = time.monotonic()

    def _on_feedback(self, _feedback) -> None:
        self._saw_executing = True

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Trajectory admission failed: {error}")
            self._finish(1)
            return
        if not goal_handle.accepted:
            self.get_logger().error("Execution manager rejected the trajectory")
            self._finish(1)
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Trajectory result failed: {error}")
            self._finish(1)
            return
        result = wrapped.result
        if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self._saw_succeeded = True
            return
        self.get_logger().error(
            f"Trajectory ended with status={wrapped.status}, "
            f"error_code={result.error_code}: {result.error_string}"
        )
        self._finish(1)

    def _make_request(self, q_start: np.ndarray) -> dict:
        zeros = np.zeros(len(self.joint_names), dtype=np.float64)
        request = {
            "schema_version": 1,
            "request_id": str(uuid.uuid4()),
            "robot_model": str(self.get_parameter("robot_model").value),
            "planning_frame": str(self.get_parameter("planning_frame").value),
            "joint_names": list(self.joint_names),
            "goal_type": self.goal_type,
            "ik_candidates": self.ik_candidates,
            "ik_max_iters": self.ik_max_iters,
            "q_pos_start": q_start.copy(),
            "q_vel_start": zeros.copy(),
            "q_vel_goal": zeros.copy(),
            "q_acc_start": zeros.copy(),
            "q_acc_goal": zeros.copy(),
            "scene_id": str(self.get_parameter("scene_id").value),
            "seed": int(self.get_parameter("seed").value),
        }
        if self.goal_type == "cartesian":
            request["ee_pose_goal"] = self.ee_goal.copy()
        else:
            request["q_pos_goal"] = self.q_goal.copy()
        return request

    def _interfaces_ready(self) -> bool:
        return self._trajectory_client.server_is_ready()

    def _joint_state_is_fresh(self, now: float) -> bool:
        return (
            self._state_received_at is not None
            and now - self._state_received_at <= self.max_joint_state_age_s
        )

    def _finish(self, code: int) -> None:
        self.exit_code = code
        if rclpy.ok():
            rclpy.shutdown()

    def _tick(self) -> None:
        now = time.monotonic()
        if self._plan_future is None:
            if self.state is None or not self._joint_state_is_fresh(now):
                return
            self._planned_from = self.state.copy()
            request = self._make_request(self._planned_from)
            self._plan_future = self._worker.submit(self.planner.plan, request)
            self.get_logger().info(
                f"Planning request {request['request_id']} in MPD worker..."
            )
            return

        if self._plan_result is None:
            if not self._plan_future.done():
                return
            try:
                raw_result = self._plan_future.result()
                assert self._planned_from is not None
                validated = validate_mpd_result(
                    raw_result,
                    expected_joint_names=self.joint_names,
                    q_start=self._planned_from,
                )
                validated["output_dir"] = raw_result["output_dir"]
                validated["result_payload"] = raw_result["result_payload"]
                self._plan_result = validated
            except Exception as error:
                self.get_logger().error(
                    f"MPD planning failed: {type(error).__name__}: {error}"
                )
                self._finish(1)
                return

            positions = self._plan_result["positions"]
            times = self._plan_result["time_from_start"]
            terminal_cartesian_pose = self._plan_result["terminal_cartesian_pose_xyzw"]
            goal_payload = self._plan_result["result_payload"]["goal"]
            target_pose = np.asarray(goal_payload["target_pose_xyzw"], dtype=np.float64)
            if target_pose.shape != (7,) or not np.isfinite(target_pose).all():
                self.get_logger().error(
                    "MPD result contains an invalid Cartesian target"
                )
                self._finish(1)
                return

            self._expected_duration_s = float(times[-1])
            output_dir = self._plan_result["output_dir"]
            self.get_logger().info(
                f"MPD plan validated: T={positions.shape[0]}, "
                f"duration={self._expected_duration_s:.3f}s, "
                f"saved={output_dir}"
            )
            self.get_logger().info(
                "TARGET_CARTESIAN_POSE_XYZW="
                + json.dumps(
                    {
                        "reference_frame": "fr3_link0",
                        "end_effector_frame": "fr3_hand",
                        "pose_xyzw": target_pose.tolist(),
                    },
                    separators=(",", ":"),
                )
            )
            self.get_logger().info(
                "BEST_TRAJECTORY_TERMINAL_CARTESIAN_POSE_XYZW="
                + json.dumps(
                    {
                        "reference_frame": "fr3_link0",
                        "end_effector_frame": "fr3_hand",
                        "pose_xyzw": terminal_cartesian_pose.tolist(),
                    },
                    separators=(",", ":"),
                )
            )
            if self.goal_type == "joint":
                q_condition = np.asarray(
                    goal_payload["q_pos_goal_condition"], dtype=np.float64
                )
                goal_seed_error = float(np.max(np.abs(positions[-1] - q_condition)))
                self.get_logger().warning(
                    "Joint input defines the target EE pose; the terminal joint "
                    "configuration is an optimized IK solution "
                    f"(max difference from input={goal_seed_error:.6f} rad)"
                )
            if self.plan_only:
                self.get_logger().info("PLAN ONLY: trajectory was not submitted")
                self._finish(0)
                return
            self._ready_since = now

        if self._published_at is None:
            assert self._plan_result is not None
            assert self._planned_from is not None
            if not self._interfaces_ready():
                assert self._ready_since is not None
                if now - self._ready_since > self.wait_for_em_s:
                    self.get_logger().error("Timeout waiting for EM action server")
                    self._finish(1)
                return
            if self.state is None or not self._joint_state_is_fresh(now):
                return
            start_drift = float(np.max(np.abs(self.state - self._planned_from)))
            if start_drift > self.max_start_drift_rad:
                self.get_logger().error(
                    f"Robot moved during planning: drift={start_drift:.6f} "
                    f"rad, limit={self.max_start_drift_rad:.6f} rad"
                )
                self._finish(1)
                return

            message = self._result_to_ros(self._plan_result)
            self._saw_executing = False
            self._saw_succeeded = False
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = message
            self._trajectory_client.send_goal_async(
                goal,
                feedback_callback=self._on_feedback,
            ).add_done_callback(self._on_goal_response)
            self._published_at = now
            self.get_logger().info(
                f"Submitted MPD plan with {len(message.points)} points"
            )
            return

        if self._saw_succeeded:
            self.get_logger().info(
                "PASS: trajectory action reported SUCCEEDED "
                f"(feedback_seen={self._saw_executing})"
            )
            self._finish(0)
            return

        result_timeout = self._expected_duration_s + self.result_timeout_margin_s
        if now - self._published_at > result_timeout:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self.get_logger().error(
                f"No trajectory success within {result_timeout:.1f}s "
                f"(executing_seen={self._saw_executing})"
            )
            self._finish(1)

    def _result_to_ros(self, result: dict) -> JointTrajectory:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(result["joint_names"])
        for position, velocity, acceleration, time_s in zip(
            result["positions"],
            result["velocities"],
            result["accelerations"],
            result["time_from_start"],
            strict=True,
        ):
            point = JointTrajectoryPoint()
            point.positions = position.tolist()
            point.velocities = velocity.tolist()
            point.accelerations = acceleration.tolist()
            nanoseconds = round(float(time_s) * 1e9)
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = nanoseconds % 1_000_000_000
            message.points.append(point)
        return message


def main() -> None:
    rclpy.init()
    node = SendMpdTrajectory()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        code = node.exit_code
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
