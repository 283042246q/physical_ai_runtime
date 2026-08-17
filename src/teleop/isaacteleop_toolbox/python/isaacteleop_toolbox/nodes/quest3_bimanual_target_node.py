#!/usr/bin/env python3
"""Quest3 bimanual source node built on IsaacTeleop TeleopSession and BaseRetargeter pipeline."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import tf2_ros

from geometry_msgs.msg import PoseStamped
from isaacteleop_toolbox.retargeters import (
    BimanualRelativeConfig,
    BimanualRelativeRetargeter,
    ControllerPose,
)
from isaacteleop_toolbox.node_parameters import create_node_parameters
from isaacteleop_toolbox.ros_publishers import BimanualTargetPublisher
from isaacteleop_toolbox.runtime import OutputMetadata, run_teleop_session_loop
from isaacteleop_toolbox.session_builders import build_controllers_session_config


class Quest3BimanualTargetNode(Node):
    """Direct IsaacTeleop Quest3 source that publishes EM pose targets."""

    def __init__(self) -> None:
        super().__init__("quest3_bimanual_target")
        self._params = create_node_parameters(self)

        # TF2 listener for dynamic alignment and frame transformation
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Retargeter configuration
        retarget_config = self._build_retarget_config()
        self.retargeter = BimanualRelativeRetargeter(
            retarget_config, on_activate_fn=self._get_robot_feedback_poses
        )
        self.publisher = BimanualTargetPublisher(
            self,
            status_topic=self._params.status_topic,
            left_target_frame=self._params.left_target_frame,
            right_target_frame=self._params.right_target_frame,
            profile_name=self._params.profile_name,
            pose_source=self._params.pose_source,
            deadman_source=self._params.deadman_source,
            left_output_topic=self._params.left_output_topic,
            right_output_topic=self._params.right_output_topic,
            left_snapshot_controller_topic=self._params.left_snapshot_controller_topic,
            left_snapshot_ee_topic=self._params.left_snapshot_ee_topic,
            right_snapshot_controller_topic=self._params.right_snapshot_controller_topic,
            right_snapshot_ee_topic=self._params.right_snapshot_ee_topic,
        )
        self.session_config = build_controllers_session_config(
            app_name="Quest3BimanualTeleopSource",
            mode=self._params.session_mode,
            mcap_config=self._params.mcap_config,
            retargeter=self.retargeter,
        )
        self.get_logger().info(
            "Quest3 bimanual source ready: "
            f"profile={self._params.profile_name} mode={self._params.session_mode.value} "
            f"left_topic={self._params.left_output_topic} right_topic={self._params.right_output_topic}"
        )

    @property
    def runtime_params(self):
        # Compatibility with runtime.py run_teleop_session_loop
        return self._params

    def _build_retarget_config(self) -> BimanualRelativeConfig:
        return BimanualRelativeConfig(
            pose_source=self._params.pose_source,
            deadman_source=self._params.deadman_source,
            deadman_threshold=self._params.deadman_threshold,
            require_both_deadman=self._params.require_both_deadman,
            linear_scale=self._params.linear_scale,
            angular_scale=self._params.angular_scale,
            lowpass_alpha=self._params.lowpass_alpha,
            max_linear_step_m=self._params.max_linear_step_m,
            max_angular_step_rad=self._params.max_angular_step_rad,
            left_anchor_position=self._params.left_anchor_position,
            right_anchor_position=self._params.right_anchor_position,
            anchor_orientation_xyzw=self._params.anchor_orientation_xyzw,
            openxr_to_base_rotation_xyzw=self._params.openxr_to_base_rotation_xyzw,
        )

    def run(self) -> int:
        return run_teleop_session_loop(
            self, self.session_config, self._publish_step
        )

    def _get_robot_feedback_poses(self) -> tuple[ControllerPose | None, ControllerPose | None]:
        if not self._params.left_flange_frame or not self._params.right_flange_frame:
            return None, None
        
        try:
            # Look up left and right flange relative to output_frame (e.g. world)
            left_tf = self._tf_buffer.lookup_transform(
                self._params.output_frame, self._params.left_flange_frame, rclpy.time.Time()
            )
            right_tf = self._tf_buffer.lookup_transform(
                self._params.output_frame, self._params.right_flange_frame, rclpy.time.Time()
            )
            
            left_pose = ControllerPose(
                position=np.array([
                    left_tf.transform.translation.x,
                    left_tf.transform.translation.y,
                    left_tf.transform.translation.z
                ], dtype=float),
                rotation=Rotation.from_quat([
                    left_tf.transform.rotation.x,
                    left_tf.transform.rotation.y,
                    left_tf.transform.rotation.z,
                    left_tf.transform.rotation.w
                ])
            )
            right_pose = ControllerPose(
                position=np.array([
                    right_tf.transform.translation.x,
                    right_tf.transform.translation.y,
                    right_tf.transform.translation.z
                ], dtype=float),
                rotation=Rotation.from_quat([
                    right_tf.transform.rotation.x,
                    right_tf.transform.rotation.y,
                    right_tf.transform.rotation.z,
                    right_tf.transform.rotation.w
                ])
            )
            self.get_logger().info(
                f"Dynamic alignment: successfully aligned targets to feedback: "
                f"L={self._params.left_flange_frame}, R={self._params.right_flange_frame} (relative to {self._params.output_frame})"
            )
            return left_pose, right_pose
        except Exception as e:
            self.get_logger().warn(
                f"Failed to lookup robot feedback poses for alignment: {e}. "
                "Using default configured anchors."
            )
            return None, None

    def _publish_step(
        self, session_result: dict, stamp, metadata: OutputMetadata
    ) -> None:
        active_tensor = session_result["active"][0]
        active_val = int(np.from_dlpack(active_tensor)[0])
        active = active_val > 0
        
        left_pose_stamped = None
        right_pose_stamped = None
        
        if active:
            # Extract 7D target pose tensors [x, y, z, qx, qy, qz, qw]
            left_ee = np.from_dlpack(session_result["left_ee_pose"][0])
            right_ee = np.from_dlpack(session_result["right_ee_pose"][0])
            
            left_pos = left_ee[:3]
            left_rot = Rotation.from_quat(left_ee[3:7])
            right_pos = right_ee[:3]
            right_rot = Rotation.from_quat(right_ee[3:7])

            # 1. Transform Left Arm target from output_frame to left_base_frame
            if self._params.left_output_topic and self._params.left_base_frame:
                left_pose_stamped = PoseStamped()
                left_pose_stamped.header.stamp = stamp
                left_pose_stamped.header.frame_id = self._params.left_base_frame
                
                try:
                    left_trans = self._tf_buffer.lookup_transform(
                        self._params.left_base_frame, self._params.output_frame, rclpy.time.Time()
                    )
                    t = left_trans.transform.translation
                    r = left_trans.transform.rotation
                    T_rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
                    T_trans = np.array([t.x, t.y, t.z])
                    
                    new_pos = T_rot.apply(left_pos) + T_trans
                    new_rot = T_rot * left_rot
                    qx, qy, qz, qw = new_rot.as_quat()
                    
                    left_pose_stamped.pose.position.x = float(new_pos[0])
                    left_pose_stamped.pose.position.y = float(new_pos[1])
                    left_pose_stamped.pose.position.z = float(new_pos[2])
                    left_pose_stamped.pose.orientation.x = float(qx)
                    left_pose_stamped.pose.orientation.y = float(qy)
                    left_pose_stamped.pose.orientation.z = float(qz)
                    left_pose_stamped.pose.orientation.w = float(qw)
                except Exception as e:
                    self.get_logger().warn(
                        f"Failed to transform left target pose to {self._params.left_base_frame}: {e}"
                    )
                    left_pose_stamped = None

            # 2. Transform Right Arm target from output_frame to right_base_frame
            if self._params.right_output_topic and self._params.right_base_frame:
                right_pose_stamped = PoseStamped()
                right_pose_stamped.header.stamp = stamp
                right_pose_stamped.header.frame_id = self._params.right_base_frame
                
                try:
                    right_trans = self._tf_buffer.lookup_transform(
                        self._params.right_base_frame, self._params.output_frame, rclpy.time.Time()
                    )
                    t = right_trans.transform.translation
                    r = right_trans.transform.rotation
                    T_rot = Rotation.from_quat([r.x, r.y, r.z, r.w])
                    T_trans = np.array([t.x, t.y, t.z])
                    
                    new_pos = T_rot.apply(right_pos) + T_trans
                    new_rot = T_rot * right_rot
                    qx, qy, qz, qw = new_rot.as_quat()
                    
                    right_pose_stamped.pose.position.x = float(new_pos[0])
                    right_pose_stamped.pose.position.y = float(new_pos[1])
                    right_pose_stamped.pose.position.z = float(new_pos[2])
                    right_pose_stamped.pose.orientation.x = float(qx)
                    right_pose_stamped.pose.orientation.y = float(qy)
                    right_pose_stamped.pose.orientation.z = float(qz)
                    right_pose_stamped.pose.orientation.w = float(qw)
                except Exception as e:
                    self.get_logger().warn(
                        f"Failed to transform right target pose to {self._params.right_base_frame}: {e}"
                    )
                    right_pose_stamped = None

        snapshot = self.retargeter.snapshot
        self.publisher.publish(
            active=active,
            initialized=active,
            left_pose_stamped=left_pose_stamped,
            right_pose_stamped=right_pose_stamped,
            snapshot_seq=snapshot.seq,
            reason="" if active else "inactive_or_invalid",
            snapshot=snapshot,
            snapshot_stamp=stamp,
            snapshot_frame_id=self._params.output_frame,
            metadata=metadata,
        )


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = Quest3BimanualTargetNode()
        return node.run()
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
