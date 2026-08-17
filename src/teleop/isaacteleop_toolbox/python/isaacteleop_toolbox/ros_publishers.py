"""ROS publishers for workspace teleop source contracts without backward-compatibility code."""

from __future__ import annotations

import json

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .retargeters import BimanualSnapshot, ControllerPose
from .runtime import OutputMetadata


STATUS_SCHEMA_VERSION = 1


def _controller_pose_to_msg(stamp, frame_id: str, pose: ControllerPose) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(pose.position[0])
    msg.pose.position.y = float(pose.position[1])
    msg.pose.position.z = float(pose.position[2])
    qx, qy, qz, qw = pose.rotation.as_quat()
    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)
    return msg


class BimanualTargetPublisher:
    """Publish EM pose targets, status, target TFs, and clutch snapshots."""

    def __init__(
        self,
        node,
        *,
        status_topic: str,
        left_target_frame: str,
        right_target_frame: str,
        profile_name: str,
        pose_source: str,
        deadman_source: str,
        left_output_topic: str,
        right_output_topic: str,
        left_snapshot_controller_topic: str,
        left_snapshot_ee_topic: str,
        right_snapshot_controller_topic: str,
        right_snapshot_ee_topic: str,
    ) -> None:
        self._node = node
        self._left_target_frame = left_target_frame
        self._right_target_frame = right_target_frame
        self._profile_name = profile_name
        self._pose_source = pose_source
        self._deadman_source = deadman_source
        
        self._pub_status = node.create_publisher(String, status_topic, 10)
        self._pub_left_target = node.create_publisher(PoseStamped, left_output_topic, 10)
        self._pub_right_target = node.create_publisher(PoseStamped, right_output_topic, 10)
        self._pub_left_snapshot_controller = node.create_publisher(
            PoseStamped, left_snapshot_controller_topic, 10
        )
        self._pub_left_snapshot_ee = node.create_publisher(
            PoseStamped, left_snapshot_ee_topic, 10
        )
        self._pub_right_snapshot_controller = node.create_publisher(
            PoseStamped, right_snapshot_controller_topic, 10
        )
        self._pub_right_snapshot_ee = node.create_publisher(
            PoseStamped, right_snapshot_ee_topic, 10
        )
        self._tf_broadcaster = TransformBroadcaster(node)

        self.left_output_topic = left_output_topic
        self.right_output_topic = right_output_topic

    def publish(
        self,
        active: bool,
        initialized: bool,
        left_pose_stamped: PoseStamped | None,
        right_pose_stamped: PoseStamped | None,
        snapshot_seq: int,
        reason: str = "",
        snapshot: BimanualSnapshot | None = None,
        snapshot_stamp=None,
        snapshot_frame_id: str = "",
        metadata: OutputMetadata | None = None,
    ) -> None:
        # Publish separate PoseStamped
        if left_pose_stamped is not None:
            self._pub_left_target.publish(left_pose_stamped)
            self._tf_broadcaster.sendTransform(
                self._make_tf(
                    left_pose_stamped.header.stamp,
                    left_pose_stamped.header.frame_id,
                    self._left_target_frame,
                    left_pose_stamped.pose,
                )
            )
        if right_pose_stamped is not None:
            self._pub_right_target.publish(right_pose_stamped)
            self._tf_broadcaster.sendTransform(
                self._make_tf(
                    right_pose_stamped.header.stamp,
                    right_pose_stamped.header.frame_id,
                    self._right_target_frame,
                    right_pose_stamped.pose,
                )
            )

        if active and snapshot is not None and snapshot_stamp is not None:
            self._publish_snapshot(snapshot, snapshot_stamp, snapshot_frame_id)

        self._publish_status(
            active,
            initialized,
            snapshot_seq,
            reason,
            metadata=metadata,
        )

    def _publish_snapshot(
        self, snapshot: BimanualSnapshot, stamp, frame_id: str
    ) -> None:
        """Publish the clutch snapshot latched at the last deadman-press.

        Held constant (same values) every cycle for the duration of one
        deadman-press episode, so an episode recorder started anywhere within
        that window observes a fresh message on each snapshot topic.
        """
        if snapshot.left_controller is not None:
            self._pub_left_snapshot_controller.publish(
                _controller_pose_to_msg(stamp, frame_id, snapshot.left_controller)
            )
        if snapshot.left_ee is not None:
            self._pub_left_snapshot_ee.publish(
                _controller_pose_to_msg(stamp, frame_id, snapshot.left_ee)
            )
        if snapshot.right_controller is not None:
            self._pub_right_snapshot_controller.publish(
                _controller_pose_to_msg(stamp, frame_id, snapshot.right_controller)
            )
        if snapshot.right_ee is not None:
            self._pub_right_snapshot_ee.publish(
                _controller_pose_to_msg(stamp, frame_id, snapshot.right_ee)
            )

    def _make_tf(
        self, stamp, parent_frame: str, child_frame: str, pose
    ) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = float(pose.position.x)
        msg.transform.translation.y = float(pose.position.y)
        msg.transform.translation.z = float(pose.position.z)
        msg.transform.rotation.x = float(pose.orientation.x)
        msg.transform.rotation.y = float(pose.orientation.y)
        msg.transform.rotation.z = float(pose.orientation.z)
        msg.transform.rotation.w = float(pose.orientation.w)
        return msg

    def _publish_status(
        self,
        active: bool,
        initialized: bool,
        snapshot_seq: int,
        reason: str,
        *,
        metadata: OutputMetadata | None,
    ) -> None:
        msg = String()
        payload = {
                "schema_version": STATUS_SCHEMA_VERSION,
                "active": active,
                "initialized": initialized,
                "profile": self._profile_name,
                "mode": "relative",
                "pose_source": self._pose_source,
                "deadman_source": self._deadman_source,
                "left_output_topic": self.left_output_topic,
                "right_output_topic": self.right_output_topic,
                "snapshot_seq": snapshot_seq,
                "reason": reason,
            }
        if metadata is not None:
            payload.update(metadata.as_dict())
        msg.data = json.dumps(
            payload,
            separators=(",", ":"),
        )
        self._pub_status.publish(msg)
