"""ROS parameter declaration, resolution, and validation for Quest3BimanualTargetNode."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node

from isaacteleop.deviceio import McapReplayConfig
from isaacteleop.teleop_session_manager import SessionMode


@dataclass(frozen=True)
class CloudXRParams:
    install_dir: str
    env_config: str | None
    accept_eula: bool
    setup_oob: bool
    usb_local: bool
    host_client: bool
    device_profile: str


@dataclass(frozen=True)
class NodeParameters:
    profile_name: str
    sleep_period_s: float
    session_mode: SessionMode
    mcap_config: McapReplayConfig | None
    cloudxr_params: CloudXRParams
    
    # Target topic names
    left_output_topic: str
    right_output_topic: str
    debug_output_topic: str
    status_topic: str

    # Clutch snapshot topic names (published while active; latch value held
    # constant for the duration of one deadman-press episode).
    left_snapshot_controller_topic: str
    left_snapshot_ee_topic: str
    right_snapshot_controller_topic: str
    right_snapshot_ee_topic: str
    
    # TF frame names
    left_base_frame: str
    right_base_frame: str
    left_flange_frame: str
    right_flange_frame: str
    output_frame: str
    left_target_frame: str
    right_target_frame: str
    
    # Relative retargeting config parameters
    pose_source: str
    deadman_source: str
    deadman_threshold: float
    require_both_deadman: bool
    linear_scale: float
    angular_scale: float
    lowpass_alpha: float
    max_linear_step_m: float
    max_angular_step_rad: float
    left_anchor_position: list[float]
    right_anchor_position: list[float]
    anchor_orientation_xyzw: list[float]
    openxr_to_base_rotation_xyzw: list[float]

    @property
    def cloudxr(self) -> CloudXRParams:
        return self.cloudxr_params


def create_node_parameters(node: Node) -> NodeParameters:
    profile_name = node.declare_parameter("profile_name", "quest3_bimanual_relative").value
    
    # Rate and MCAP
    rate_hz = _load_rate_hz(node)
    session_mode, mcap_config = _load_mcap_replay(node)
    cloudxr_params = _load_cloudxr(node)
    
    # Topics
    left_output_topic = node.declare_parameter("left_output_topic", "").value
    right_output_topic = node.declare_parameter("right_output_topic", "").value
    debug_output_topic = node.declare_parameter("debug_output_topic", "/teleop/quest3_bimanual/pose_chunk").value
    status_topic = node.declare_parameter("status_topic", "/teleop/quest3_bimanual/status").value
    left_snapshot_controller_topic = node.declare_parameter(
        "left_snapshot_controller_topic", "/teleop/quest3_bimanual/left_snapshot/controller_pose"
    ).value
    left_snapshot_ee_topic = node.declare_parameter(
        "left_snapshot_ee_topic", "/teleop/quest3_bimanual/left_snapshot/ee_pose"
    ).value
    right_snapshot_controller_topic = node.declare_parameter(
        "right_snapshot_controller_topic", "/teleop/quest3_bimanual/right_snapshot/controller_pose"
    ).value
    right_snapshot_ee_topic = node.declare_parameter(
        "right_snapshot_ee_topic", "/teleop/quest3_bimanual/right_snapshot/ee_pose"
    ).value
    
    # TF Frames
    left_base_frame = node.declare_parameter("left_base_frame", "").value
    right_base_frame = node.declare_parameter("right_base_frame", "").value
    left_flange_frame = node.declare_parameter("left_flange_frame", "").value
    right_flange_frame = node.declare_parameter("right_flange_frame", "").value
    output_frame = node.declare_parameter("output_frame", "world").value
    left_target_frame = node.declare_parameter("left_target_frame", "teleop_left_ee_target").value
    right_target_frame = node.declare_parameter("right_target_frame", "teleop_right_ee_target").value
    
    # Retargeting parameters
    pose_source = node.declare_parameter("pose_source", "grip").value
    deadman_source = node.declare_parameter("deadman_source", "squeeze").value
    deadman_threshold = float(node.declare_parameter("deadman_threshold", 0.5).value)
    require_both_deadman = bool(node.declare_parameter("require_both_deadman", True).value)
    linear_scale = float(node.declare_parameter("linear_scale", 1.0).value)
    angular_scale = float(node.declare_parameter("angular_scale", 1.0).value)
    lowpass_alpha = float(node.declare_parameter("lowpass_alpha", 0.35).value)
    max_linear_step_m = float(node.declare_parameter("max_linear_step_m", 0.03).value)
    max_angular_step_rad = float(node.declare_parameter("max_angular_step_rad", 0.15).value)
    
    left_anchor = node.declare_parameter("left_anchor_position", [-0.30, 0.25, 0.60]).value
    right_anchor = node.declare_parameter("right_anchor_position", [0.30, 0.25, 0.60]).value
    anchor_quat = node.declare_parameter("anchor_orientation_xyzw", [0.0, 0.0, 0.0, 1.0]).value
    openxr_to_base_quat = node.declare_parameter(
        "openxr_to_base_rotation_xyzw", [0.0, 0.0, 0.0, 1.0]
    ).value

    _validate_retarget_parameters(
        pose_source=pose_source,
        deadman_source=deadman_source,
        deadman_threshold=deadman_threshold,
        linear_scale=linear_scale,
        angular_scale=angular_scale,
        lowpass_alpha=lowpass_alpha,
        max_linear_step_m=max_linear_step_m,
        max_angular_step_rad=max_angular_step_rad,
        left_anchor=left_anchor,
        right_anchor=right_anchor,
        anchor_quat=anchor_quat,
        openxr_to_base_quat=openxr_to_base_quat,
    )
    
    return NodeParameters(
        profile_name=profile_name,
        sleep_period_s=1.0 / rate_hz,
        session_mode=session_mode,
        mcap_config=mcap_config,
        cloudxr_params=cloudxr_params,
        left_output_topic=left_output_topic,
        right_output_topic=right_output_topic,
        debug_output_topic=debug_output_topic,
        status_topic=status_topic,
        left_snapshot_controller_topic=left_snapshot_controller_topic,
        left_snapshot_ee_topic=left_snapshot_ee_topic,
        right_snapshot_controller_topic=right_snapshot_controller_topic,
        right_snapshot_ee_topic=right_snapshot_ee_topic,
        left_base_frame=left_base_frame,
        right_base_frame=right_base_frame,
        left_flange_frame=left_flange_frame,
        right_flange_frame=right_flange_frame,
        output_frame=output_frame,
        left_target_frame=left_target_frame,
        right_target_frame=right_target_frame,
        pose_source=pose_source,
        deadman_source=deadman_source,
        deadman_threshold=deadman_threshold,
        require_both_deadman=require_both_deadman,
        linear_scale=linear_scale,
        angular_scale=angular_scale,
        lowpass_alpha=lowpass_alpha,
        max_linear_step_m=max_linear_step_m,
        max_angular_step_rad=max_angular_step_rad,
        left_anchor_position=list(left_anchor),
        right_anchor_position=list(right_anchor),
        anchor_orientation_xyzw=list(anchor_quat),
        openxr_to_base_rotation_xyzw=list(openxr_to_base_quat),
    )


def _load_rate_hz(node: Node) -> float:
    node.declare_parameter("rate_hz", 60.0)
    rate_hz = node.get_parameter("rate_hz").get_parameter_value().double_value
    if rate_hz <= 0 or not math.isfinite(rate_hz):
        raise ValueError("Parameter 'rate_hz' must be > 0")
    return rate_hz


def _load_mcap_replay(
    node: Node,
) -> tuple[SessionMode, McapReplayConfig | None]:
    node.declare_parameter(
        "mcap_replay_path",
        "",
        ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Optional MCAP file to replay through TeleopSession.",
        ),
    )
    mcap_replay_path = (
        node.get_parameter("mcap_replay_path")
        .get_parameter_value()
        .string_value.strip()
    )
    if not mcap_replay_path:
        return SessionMode.LIVE, None

    replay_path = Path(mcap_replay_path).expanduser().resolve()
    if not replay_path.is_file():
        raise FileNotFoundError(f"mcap_replay_path file not found: {replay_path}")
    node.get_logger().info(f"Replaying MCAP input: {replay_path}")
    return SessionMode.REPLAY, McapReplayConfig(str(replay_path))


def _get_bool_param(node: Node, name: str) -> bool:
    val = node.get_parameter(name).value
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        return val.lower() in ("true", "1", "yes")
    return bool(val)


def _validate_retarget_parameters(
    *,
    pose_source,
    deadman_source,
    deadman_threshold,
    linear_scale,
    angular_scale,
    lowpass_alpha,
    max_linear_step_m,
    max_angular_step_rad,
    left_anchor,
    right_anchor,
    anchor_quat,
    openxr_to_base_quat,
) -> None:
    if pose_source not in {"aim", "grip"}:
        raise ValueError("pose_source must be 'aim' or 'grip'")
    valid_deadman = {"none", "squeeze", "trigger", "primary", "secondary", "thumbstick"}
    if deadman_source not in valid_deadman:
        raise ValueError(f"deadman_source must be one of {sorted(valid_deadman)}")
    if not math.isfinite(deadman_threshold) or not 0.0 <= deadman_threshold <= 1.0:
        raise ValueError("deadman_threshold must be finite and in [0, 1]")

    nonnegative = {
        "linear_scale": linear_scale,
        "angular_scale": angular_scale,
        "max_linear_step_m": max_linear_step_m,
        "max_angular_step_rad": max_angular_step_rad,
    }
    for name, value in nonnegative.items():
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and >= 0")
    if not math.isfinite(lowpass_alpha) or not 0.0 <= lowpass_alpha <= 1.0:
        raise ValueError("lowpass_alpha must be finite and in [0, 1]")

    for name, vector, length in (
        ("left_anchor_position", left_anchor, 3),
        ("right_anchor_position", right_anchor, 3),
        ("anchor_orientation_xyzw", anchor_quat, 4),
        ("openxr_to_base_rotation_xyzw", openxr_to_base_quat, 4),
    ):
        if len(vector) != length or not all(math.isfinite(float(v)) for v in vector):
            raise ValueError(f"{name} must contain {length} finite values")

    for name, quaternion in (
        ("anchor_orientation_xyzw", anchor_quat),
        ("openxr_to_base_rotation_xyzw", openxr_to_base_quat),
    ):
        norm = math.sqrt(sum(float(v) ** 2 for v in quaternion))
        if not math.isclose(norm, 1.0, rel_tol=1e-5, abs_tol=1e-5):
            raise ValueError(f"{name} must be a normalized nonzero quaternion")


def _load_cloudxr(node: Node) -> CloudXRParams:
    default_cloudxr_dir = os.environ.get("CLOUDXR_DIR", "")
    node.declare_parameter("cloudxr_install_dir", default_cloudxr_dir)
    node.declare_parameter("cloudxr_env_config", "")
    node.declare_parameter("cloudxr_accept_eula", False)
    node.declare_parameter("cloudxr_setup_oob", False)
    node.declare_parameter("cloudxr_usb_local", False)
    node.declare_parameter("cloudxr_host_client", False)
    node.declare_parameter("cloudxr_device_profile", "Quest3")

    install_dir = (
        node.get_parameter("cloudxr_install_dir")
        .get_parameter_value()
        .string_value.strip()
    )
    if not install_dir:
        raise ValueError(
            "cloudxr_install_dir must be set explicitly outside the Physical AI Runtime "
            "Pixi environment"
        )
    install_dir = str(Path(install_dir).expanduser().resolve())
    
    env_config_str = (
        node.get_parameter("cloudxr_env_config")
        .get_parameter_value()
        .string_value.strip()
    )
    env_config = None
    if env_config_str:
        env_config_path = Path(env_config_str).expanduser()
        if env_config_path.is_file():
            env_config = str(env_config_path)
        else:
            node.get_logger().warn(
                f"cloudxr_env_config file not found at {env_config_path}, ignoring."
            )

    setup_oob = _get_bool_param(node, "cloudxr_setup_oob")
    usb_local = _get_bool_param(node, "cloudxr_usb_local")
    if usb_local and not setup_oob:
        raise ValueError(
            "Parameter 'cloudxr_usb_local' requires 'cloudxr_setup_oob' to be true"
        )
    
    device_profile = (
        node.get_parameter("cloudxr_device_profile")
        .get_parameter_value()
        .string_value.strip()
    ) or "Quest3"

    return CloudXRParams(
        install_dir=install_dir,
        env_config=env_config,
        accept_eula=_get_bool_param(node, "cloudxr_accept_eula"),
        setup_oob=setup_oob,
        usb_local=usb_local,
        host_client=_get_bool_param(node, "cloudxr_host_client"),
        device_profile=device_profile,
    )
