"""Relative bimanual target retargeting from IsaacTeleop controller tensors."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from isaacteleop.retargeting_engine.interface import (
    BaseRetargeter,
    OptionalTensorGroup,
    RetargeterIOType,
)
from isaacteleop.retargeting_engine.interface.retargeter_core_types import RetargeterIO
from isaacteleop.retargeting_engine.interface.tensor_group_type import (
    TensorGroupType,
    OptionalType,
)
from isaacteleop.retargeting_engine.tensor_types import (
    ControllerInput,
    NDArrayType,
    DLDataType,
    ControllerInputIndex,
)


@dataclass
class ControllerPose:
    position: np.ndarray
    rotation: Rotation


@dataclass
class SideState:
    source_initial: ControllerPose | None = None
    target_initial: ControllerPose | None = None
    previous_target: ControllerPose | None = None


@dataclass(frozen=True)
class BimanualSnapshot:
    """Clutch snapshot latched on the most recent deadman-press transition.

    All poses are expressed in the same frame as the retargeter output
    (``output_frame``, e.g. "world"): ``left_controller``/``right_controller``
    are the controller poses (in robot-base-aligned axes) at the moment the
    snapshot was taken; ``left_ee``/``right_ee`` are the robot EE poses the
    relative delta is anchored to (dynamic FK feedback, or the static anchor
    fallback). Recording this alongside the teleop targets lets a replay or
    offline analysis recover the exact reference used for that episode's
    delta math, without re-deriving it from TF history.
    """

    seq: int
    left_controller: ControllerPose | None
    left_ee: ControllerPose | None
    right_controller: ControllerPose | None
    right_ee: ControllerPose | None


@dataclass(frozen=True)
class BimanualRelativeConfig:
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
    # Fixed basis-change rotation applied to raw controller poses before any
    # delta math, converting OpenXR-frame axes into robot-base-frame axes.
    # Identity [0, 0, 0, 1] means no-op (OpenXR and base axes assumed aligned).
    openxr_to_base_rotation_xyzw: list[float]


def _slerp(start: Rotation, end: Rotation, alpha: float) -> Rotation:
    return Slerp([0.0, 1.0], Rotation.concatenate([start, end]))([alpha])[0]


def _clamp_rotation_step(previous: Rotation, target: Rotation, max_step_rad: float) -> Rotation:
    if max_step_rad <= 0.0:
        return target
    delta = target * previous.inv()
    rotvec = delta.as_rotvec()
    angle = float(np.linalg.norm(rotvec))
    if angle <= max_step_rad or angle == 0.0:
        return target
    limited_delta = Rotation.from_rotvec(rotvec * (max_step_rad / angle))
    return limited_delta * previous


def _apply_basis_change(pose: ControllerPose, basis_change: Rotation) -> ControllerPose:
    """Re-express a pose's axes under a different basis (translation unaffected).

    Equivalent to composing the pose with a pure-rotation frame change:
    position' = R @ position, rotation' = R * rotation. Used to convert a
    controller pose read directly in the OpenXR frame into the equivalent
    representation in robot-base-aligned axes, before any delta computation.
    """
    return ControllerPose(
        position=basis_change.apply(pose.position),
        rotation=basis_change * pose.rotation,
    )


def _controller_pose(ctrl: OptionalTensorGroup, pose_source: str) -> ControllerPose:
    if pose_source == "grip":
        position_index = ControllerInputIndex.GRIP_POSITION
        orientation_index = ControllerInputIndex.GRIP_ORIENTATION
    elif pose_source == "aim":
        position_index = ControllerInputIndex.AIM_POSITION
        orientation_index = ControllerInputIndex.AIM_ORIENTATION
    else:
        raise ValueError("pose_source must be 'grip' or 'aim'")

    return ControllerPose(
        position=np.from_dlpack(ctrl[position_index]),
        rotation=Rotation.from_quat(np.from_dlpack(ctrl[orientation_index])),
    )


def _scalar_value(value) -> float:
    if hasattr(value, "__dlpack__"):
        return float(np.from_dlpack(value))
    return float(value)


def _pose_valid(ctrl: OptionalTensorGroup, pose_source: str) -> bool:
    if ctrl.is_none:
        return False
    if pose_source == "grip":
        return bool(_scalar_value(ctrl[ControllerInputIndex.GRIP_IS_VALID]))
    if pose_source == "aim":
        return bool(_scalar_value(ctrl[ControllerInputIndex.AIM_IS_VALID]))
    raise ValueError("pose_source must be 'grip' or 'aim'")


def _deadman_pressed(
    ctrl: OptionalTensorGroup, source: str, threshold: float
) -> bool:
    if ctrl.is_none:
        return False
    if source == "none":
        return True
    if source == "squeeze":
        return _scalar_value(ctrl[ControllerInputIndex.SQUEEZE_VALUE]) >= threshold
    if source == "trigger":
        return _scalar_value(ctrl[ControllerInputIndex.TRIGGER_VALUE]) >= threshold
    if source == "primary":
        return _scalar_value(ctrl[ControllerInputIndex.PRIMARY_CLICK]) >= threshold
    if source == "secondary":
        return _scalar_value(ctrl[ControllerInputIndex.SECONDARY_CLICK]) >= threshold
    if source == "thumbstick":
        return _scalar_value(ctrl[ControllerInputIndex.THUMBSTICK_CLICK]) >= threshold
    raise ValueError(
        "deadman_source must be one of: none, squeeze, trigger, primary, secondary, thumbstick"
    )


class BimanualRelativeRetargeter(BaseRetargeter):
    """Pipeline-integrated stateful clutch-relative mapping from two controllers to two EE targets."""

    def __init__(self, config: BimanualRelativeConfig, on_activate_fn=None, name: str = "bimanual_relative") -> None:
        self.config = config
        self.on_activate_fn = on_activate_fn
        anchor_rotation = Rotation.from_quat(np.asarray(config.anchor_orientation_xyzw))
        self._left = SideState(
            target_initial=ControllerPose(
                np.asarray(config.left_anchor_position, dtype=float), anchor_rotation
            )
        )
        self._right = SideState(
            target_initial=ControllerPose(
                np.asarray(config.right_anchor_position, dtype=float), anchor_rotation
            )
        )
        self._active = False
        self.snapshot_seq = 0
        self._openxr_to_base = Rotation.from_quat(
            np.asarray(config.openxr_to_base_rotation_xyzw, dtype=float)
        )
        super().__init__(name=name)

    @property
    def snapshot(self) -> BimanualSnapshot:
        """Current clutch snapshot (latched at the last deadman-press transition)."""
        return BimanualSnapshot(
            seq=self.snapshot_seq,
            left_controller=self._left.source_initial,
            left_ee=self._left.target_initial,
            right_controller=self._right.source_initial,
            right_ee=self._right.target_initial,
        )

    def input_spec(self) -> RetargeterIOType:
        return {
            "controller_left": OptionalType(ControllerInput()),
            "controller_right": OptionalType(ControllerInput()),
        }

    def output_spec(self) -> RetargeterIOType:
        return {
            "left_ee_pose": TensorGroupType(
                "left_ee_pose",
                [
                    NDArrayType(
                        "pose", shape=(7,), dtype=DLDataType.FLOAT, dtype_bits=32
                    )
                ],
            ),
            "right_ee_pose": TensorGroupType(
                "right_ee_pose",
                [
                    NDArrayType(
                        "pose", shape=(7,), dtype=DLDataType.FLOAT, dtype_bits=32
                    )
                ],
            ),
            "active": TensorGroupType(
                "active",
                [
                    NDArrayType(
                        "flag", shape=(1,), dtype=DLDataType.INT, dtype_bits=32
                    )
                ],
            ),
        }

    def _compute_fn(self, inputs: RetargeterIO, outputs: RetargeterIO, context) -> None:
        if context.execution_events.reset:
            self._active = False
            self._left.source_initial = None
            self._right.source_initial = None

        left_ctrl = inputs["controller_left"]
        right_ctrl = inputs["controller_right"]
        left_ee_pose = outputs["left_ee_pose"]
        right_ee_pose = outputs["right_ee_pose"]
        active_out = outputs["active"]

        left_valid = _pose_valid(left_ctrl, self.config.pose_source)
        right_valid = _pose_valid(right_ctrl, self.config.pose_source)
        left_deadman = _deadman_pressed(
            left_ctrl, self.config.deadman_source, self.config.deadman_threshold
        )
        right_deadman = _deadman_pressed(
            right_ctrl, self.config.deadman_source, self.config.deadman_threshold
        )

        if self.config.require_both_deadman:
            active = left_valid and right_valid and left_deadman and right_deadman
        else:
            active = (left_valid and left_deadman) or (right_valid and right_deadman)

        if not active or not left_valid or not right_valid:
            self._active = False
            self._left.source_initial = None
            self._right.source_initial = None
            left_ee_pose[0] = np.zeros(7, dtype=np.float32)
            right_ee_pose[0] = np.zeros(7, dtype=np.float32)
            active_out[0] = np.array([0], dtype=np.int32)
            return

        left_source = _apply_basis_change(
            _controller_pose(left_ctrl, self.config.pose_source), self._openxr_to_base
        )
        right_source = _apply_basis_change(
            _controller_pose(right_ctrl, self.config.pose_source), self._openxr_to_base
        )

        if not self._active:
            self._active = True
            self.snapshot_seq += 1
            if self.on_activate_fn is not None:
                left_align, right_align = self.on_activate_fn()
                if left_align is not None:
                    self._left.target_initial = left_align
                if right_align is not None:
                    self._right.target_initial = right_align
            self._latch_snapshot(left_source, right_source)

        left_target = self._relative_target(self._left, left_source)
        right_target = self._relative_target(self._right, right_source)

        left_ee_pose[0] = np.concatenate([left_target.position, left_target.rotation.as_quat()]).astype(np.float32)
        right_ee_pose[0] = np.concatenate([right_target.position, right_target.rotation.as_quat()]).astype(np.float32)
        active_out[0] = np.array([1], dtype=np.int32)

    def _latch_snapshot(
        self, left_source: ControllerPose, right_source: ControllerPose
    ) -> None:
        self._left.source_initial = left_source
        self._right.source_initial = right_source
        self._left.previous_target = self._left.target_initial
        self._right.previous_target = self._right.target_initial

    def _relative_target(self, side: SideState, current: ControllerPose) -> ControllerPose:
        if side.source_initial is None or side.target_initial is None:
            raise RuntimeError("relative target requested before snapshot latch")

        position_delta = (
            current.position - side.source_initial.position
        ) * self.config.linear_scale
        rotation_delta = current.rotation * side.source_initial.rotation.inv()
        rotation_delta = Rotation.from_rotvec(
            rotation_delta.as_rotvec() * self.config.angular_scale
        )

        target = ControllerPose(
            position=side.target_initial.position + position_delta,
            rotation=rotation_delta * side.target_initial.rotation,
        )

        previous = side.previous_target
        alpha = float(np.clip(self.config.lowpass_alpha, 0.0, 1.0))
        if previous is not None and alpha < 1.0:
            target = ControllerPose(
                position=(alpha * target.position) + ((1.0 - alpha) * previous.position),
                rotation=_slerp(previous.rotation, target.rotation, alpha),
            )

        if previous is not None:
            step = target.position - previous.position
            step_norm = float(np.linalg.norm(step))
            if (
                self.config.max_linear_step_m > 0.0
                and step_norm > self.config.max_linear_step_m
            ):
                target.position = (
                    previous.position
                    + step * (self.config.max_linear_step_m / step_norm)
                )
            target.rotation = _clamp_rotation_step(
                previous.rotation,
                target.rotation,
                self.config.max_angular_step_rad,
            )

        side.previous_target = target
        return target
