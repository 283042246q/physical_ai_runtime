"""Resolve the workspace URDF/XML from ROS 2 `robot_description` parameters.

In ROS 2 the processed URDF (or xacro output) is typically stored as the
``robot_description`` string parameter on ``robot_state_publisher`` or
``ros2_control_node``. Planner nodes may either:

1. Receive the same parameter from launch (preferred when available), or
2. Read it from another node via ``AsyncParameterClient``.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient


def resolve_robot_description_xml(
    node: Node,
    *,
    parameter_name: str = "robot_description",
    source_node: str = "robot_state_publisher",
    timeout_s: float = 10.0,
) -> str:
    """Return URDF/XML text from a local or remote ``robot_description`` param."""

    if not node.has_parameter(parameter_name):
        node.declare_parameter(parameter_name, "")

    local_value = node.get_parameter(parameter_name).value
    if isinstance(local_value, str) and local_value.strip():
        node.get_logger().info(
            f"Using {parameter_name} from this node ({len(local_value)} bytes)."
        )
        return local_value.strip()

    if not source_node:
        raise RuntimeError(
            f"{parameter_name} is empty on this node and source_node is unset."
        )

    node.get_logger().info(
        f"Fetching {parameter_name} from '{source_node}'..."
    )
    client = AsyncParameterClient(node, source_node)
    if not client.wait_for_services(timeout_sec=timeout_s):
        raise RuntimeError(
            f"Timed out waiting for parameter services on '{source_node}'."
        )

    future = client.get_parameters([parameter_name])
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise RuntimeError(
            f"Timed out reading {parameter_name} from '{source_node}'."
        )
    if future.exception() is not None:
        raise RuntimeError(
            f"Failed to read {parameter_name} from '{source_node}': "
            f"{future.exception()}"
        )

    response = future.result()
    if response is None or not response.values:
        raise RuntimeError(f"'{source_node}' did not return {parameter_name}.")

    robot_description = parameter_value_to_python(response.values[0])
    if not isinstance(robot_description, str) or not robot_description.strip():
        raise RuntimeError(
            f"{parameter_name} from '{source_node}' is empty or not a string."
        )

    node.get_logger().info(
        f"Loaded {parameter_name} from '{source_node}' "
        f"({len(robot_description)} bytes)."
    )
    return robot_description.strip()
