"""Interactive targets published on an explicit RViz Plan request."""

from __future__ import annotations

from copy import deepcopy
from math import sqrt

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


def _quaternion(x: float, y: float, z: float, w: float) -> Quaternion:
    norm = sqrt(x * x + y * y + z * z + w * w)
    return Quaternion(x=x / norm, y=y / norm, z=z / norm, w=w / norm)


def _axis(name: str, x: float, y: float, z: float, mode: int):
    control = InteractiveMarkerControl()
    control.name = name
    control.orientation = _quaternion(x, y, z, 1.0)
    control.interaction_mode = mode
    return control


class PlanningTargetMarkers(Node):
    def __init__(self) -> None:
        super().__init__("motion_planning_target_markers")
        self._frame_id = str(self.declare_parameter("frame_id", "world").value)
        enabled_arms = str(self.declare_parameter("enabled_arms", "left").value)
        sides = [side.strip() for side in enabled_arms.split(",") if side.strip()]
        if not sides or any(side not in {"left", "right"} for side in sides):
            raise ValueError("enabled_arms must contain left and/or right")
        self._poses: dict[str, Pose] = {}
        topics = {
            "left": str(self.declare_parameter("left_topic", "/teleop/pose_left").value),
            "right": str(
                self.declare_parameter("right_topic", "/teleop/pose_right").value
            ),
        }
        self._target_publishers = {
            side: self.create_publisher(PoseStamped, topics[side], 1) for side in sides
        }
        self._server = InteractiveMarkerServer(self, "motion_planning_targets")
        self.create_subscription(Empty, "/motion_planning/plan", self._publish, 1)
        initial = {"left": [0.45, 0.25, 0.85], "right": [0.45, -0.25, 0.85]}
        for side in sides:
            self._insert_marker(side, initial[side])
        self._server.applyChanges()

    def _insert_marker(self, side: str, xyz: list[float]) -> None:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        self._poses[side] = pose
        marker = InteractiveMarker()
        marker.header.frame_id = self._frame_id
        marker.name = f"target_{side}"
        marker.description = f"{side.capitalize()} arm target"
        marker.scale = 0.25
        marker.pose = deepcopy(pose)
        visual = Marker()
        visual.type = Marker.CUBE
        visual.scale.x = 0.05
        visual.scale.y = 0.05
        visual.scale.z = 0.05
        visual.color.r = 0.1
        visual.color.g = 0.8
        visual.color.b = 0.9
        visual.color.a = 0.9
        visual_control = InteractiveMarkerControl()
        visual_control.name = "target_visual"
        visual_control.always_visible = True
        visual_control.markers.append(visual)
        marker.controls.append(visual_control)
        marker.controls.extend(
            [
                _axis("rotate_x", 1.0, 0.0, 0.0, InteractiveMarkerControl.ROTATE_AXIS),
                _axis("move_x", 1.0, 0.0, 0.0, InteractiveMarkerControl.MOVE_AXIS),
                _axis("rotate_y", 0.0, 1.0, 0.0, InteractiveMarkerControl.ROTATE_AXIS),
                _axis("move_y", 0.0, 1.0, 0.0, InteractiveMarkerControl.MOVE_AXIS),
                _axis("rotate_z", 0.0, 0.0, 1.0, InteractiveMarkerControl.ROTATE_AXIS),
                _axis("move_z", 0.0, 0.0, 1.0, InteractiveMarkerControl.MOVE_AXIS),
            ]
        )
        self._server.insert(marker, feedback_callback=self._feedback)

    def _feedback(self, feedback) -> None:
        self._poses[feedback.marker_name.removeprefix("target_")] = deepcopy(
            feedback.pose
        )

    def _publish(self, _request: Empty) -> None:
        stamp = self.get_clock().now().to_msg()
        for side, publisher in self._target_publishers.items():
            target = PoseStamped()
            target.header.stamp = stamp
            target.header.frame_id = self._frame_id
            target.pose = deepcopy(self._poses[side])
            publisher.publish(target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningTargetMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
