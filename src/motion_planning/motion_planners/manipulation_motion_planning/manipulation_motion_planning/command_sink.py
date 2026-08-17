"""Command output sinks for planner sources.

Planner backends return backend-neutral result objects. A command sink owns the
ROS publication route for those results. The default route publishes EM source
contracts; a direct route is available for adapter bring-up and hardware-less
debugging.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .contracts import HorizonPlanPoint, TrajectoryPlanPoint


REFERENCE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class CommandSink(ABC):
    """Abstract planner command output.

    Implementations may publish to EM source topics, a direct controller topic,
    or a logging/test harness. Methods that are unsupported by a sink should
    raise `NotImplementedError` rather than silently dropping commands.
    """

    @abstractmethod
    def publish_joint_target(
        self, node: Node, joint_names: list[str], positions: list[float]
    ) -> None:
        """Publish a single joint-space target."""
        ...

    @abstractmethod
    def publish_joint_chunk(
        self, node: Node, joint_names: list[str], points: list[HorizonPlanPoint]
    ) -> None:
        """Publish a receding-horizon joint chunk."""
        ...

    @abstractmethod
    def publish_joint_trajectory(
        self, node: Node, joint_names: list[str], points: list[TrajectoryPlanPoint]
    ) -> None:
        """Publish a complete joint trajectory."""
        ...


class EMCommandSink(CommandSink):
    """Publish planner outputs as EM source contracts."""

    def __init__(
        self,
        node: Node,
        *,
        source_name: str,
        source_namespace: str = "/action_sources",
        qos: QoSProfile = REFERENCE_QOS,
    ) -> None:
        source_namespace = source_namespace.rstrip("/")
        self._joint_target_pub = node.create_publisher(
            JointState,
            f"{source_namespace}/{source_name}/joint_target",
            qos,
        )
        self._joint_chunk_pub = node.create_publisher(
            JointTrajectory,
            f"{source_namespace}/{source_name}/joint_chunk",
            qos,
        )
        self._joint_trajectory_pub = node.create_publisher(
            JointTrajectory,
            f"{source_namespace}/{source_name}/joint_trajectory_goal",
            qos,
        )

    def publish_joint_target(
        self, node: Node, joint_names: list[str], positions: list[float]
    ) -> None:
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(joint_names)
        msg.position = list(positions)
        self._joint_target_pub.publish(msg)

    def publish_joint_chunk(
        self, node: Node, joint_names: list[str], points: list[HorizonPlanPoint]
    ) -> None:
        self._joint_chunk_pub.publish(
            _joint_trajectory_msg(node, joint_names, points)
        )

    def publish_joint_trajectory(
        self, node: Node, joint_names: list[str], points: list[TrajectoryPlanPoint]
    ) -> None:
        self._joint_trajectory_pub.publish(
            _joint_trajectory_msg(node, joint_names, points)
        )


class DirectRobotCommandSink(CommandSink):
    """Publish commands directly to a JointTrajectory topic.

    This is intended for adapter bring-up and debug paths where EM is not in the
    loop. It uses `JointTrajectory` for both chunks and full trajectories.
    """

    def __init__(
        self,
        node: Node,
        *,
        command_topic: str,
        qos: QoSProfile = REFERENCE_QOS,
    ) -> None:
        self._trajectory_pub = node.create_publisher(
            JointTrajectory,
            command_topic,
            qos,
        )

    def publish_joint_target(
        self, node: Node, joint_names: list[str], positions: list[float]
    ) -> None:
        point = HorizonPlanPoint(positions=list(positions), time_from_start_s=0.0)
        self.publish_joint_chunk(node, joint_names, [point])

    def publish_joint_chunk(
        self, node: Node, joint_names: list[str], points: list[HorizonPlanPoint]
    ) -> None:
        self._trajectory_pub.publish(_joint_trajectory_msg(node, joint_names, points))

    def publish_joint_trajectory(
        self, node: Node, joint_names: list[str], points: list[TrajectoryPlanPoint]
    ) -> None:
        self._trajectory_pub.publish(_joint_trajectory_msg(node, joint_names, points))


class RecordingCommandSink(CommandSink):
    """In-memory sink for tests and diagnostics."""

    def __init__(self) -> None:
        self.joint_targets: list[tuple[list[str], list[float]]] = []
        self.joint_chunks: list[tuple[list[str], list[HorizonPlanPoint]]] = []
        self.joint_trajectories: list[tuple[list[str], list[TrajectoryPlanPoint]]] = []

    def publish_joint_target(
        self, node: Node, joint_names: list[str], positions: list[float]
    ) -> None:
        self.joint_targets.append((list(joint_names), list(positions)))

    def publish_joint_chunk(
        self, node: Node, joint_names: list[str], points: list[HorizonPlanPoint]
    ) -> None:
        self.joint_chunks.append((list(joint_names), list(points)))

    def publish_joint_trajectory(
        self, node: Node, joint_names: list[str], points: list[TrajectoryPlanPoint]
    ) -> None:
        self.joint_trajectories.append((list(joint_names), list(points)))


def make_command_sink(
    node: Node,
    *,
    mode: str,
    source_name: str,
    source_namespace: str = "/action_sources",
    direct_command_topic: Optional[str] = None,
) -> CommandSink:
    """Create a command sink from a small config surface."""

    normalized = mode.lower().strip()
    if normalized == "em":
        return EMCommandSink(
            node,
            source_name=source_name,
            source_namespace=source_namespace,
        )
    if normalized == "direct":
        if not direct_command_topic:
            raise ValueError("direct_command_topic is required for direct sink mode")
        return DirectRobotCommandSink(node, command_topic=direct_command_topic)
    if normalized == "recording":
        return RecordingCommandSink()
    raise ValueError(f"Unknown command sink mode: {mode!r}")


def _joint_trajectory_msg(
    node: Node,
    joint_names: list[str],
    points: list[HorizonPlanPoint] | list[TrajectoryPlanPoint],
) -> JointTrajectory:
    msg = JointTrajectory()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.joint_names = list(joint_names)
    for point in points:
        traj_point = JointTrajectoryPoint()
        traj_point.positions = list(point.positions)
        if point.velocities is not None:
            traj_point.velocities = list(point.velocities)
        traj_point.time_from_start = Duration(
            seconds=point.time_from_start_s
        ).to_msg()
        msg.points.append(traj_point)
    return msg
