"""Tests for robot_description parameter resolution."""

from __future__ import annotations

import rclpy
from manipulation_motion_planning.robot_description import resolve_robot_description_xml


_MINIMAL_URDF = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
</robot>
"""


def test_resolve_robot_description_from_local_parameter() -> None:
    initialized_here = False
    if not rclpy.ok():
        rclpy.init()
        initialized_here = True
    node = rclpy.create_node("test_robot_description_resolver")
    try:
        node.declare_parameter("robot_description", _MINIMAL_URDF)
        resolved = resolve_robot_description_xml(node, source_node="")
        assert resolved == _MINIMAL_URDF.strip()
    finally:
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()
