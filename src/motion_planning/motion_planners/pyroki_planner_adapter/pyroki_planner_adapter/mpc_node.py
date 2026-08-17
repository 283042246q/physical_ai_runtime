"""PyRoki online MPC planner ROS node."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException

from manipulation_motion_planning.contracts import PoseTarget
from manipulation_motion_planning.planner_sources import (
    OnlineMpcPlannerSource,
    OnlineMpcSourceConfig,
)
from manipulation_motion_planning.robot_description import resolve_robot_description_xml

from . import _bootstrap  # noqa: F401
from .config import (
    PyrokiOnlineMpcNodeConfig,
    PyrokiOnlineMpcSolverConfig,
    PyrokiRobotLoadConfig,
)
from .pyroki_backend import PyrokiHorizonMpcBackend
from .robot_loader import load_robot_collision_from_urdf, load_robot_from_urdf


def _parse_name_list(value: str) -> list[str]:
    return [part.strip() for part in value.split(",") if part.strip()]


class PyrokiOnlineMpcPlannerNode(OnlineMpcPlannerSource):
    """Pose target stream -> PyRoki online MPC -> EM `joint_chunk`."""

    def __init__(self) -> None:
        super().__init__("pyroki_online_mpc_planner")
        self._load_parameters()
        self._load_robot_model()
        self._configure_planner()
        self.create_subscription(PoseStamped, self._node_config.pose_topic, self._on_pose, 10)

    def _load_parameters(self) -> None:
        self._robot_config = PyrokiRobotLoadConfig(
            target_link_name=self.declare_parameter(
                "target_link_name", "flange_L"
            ).value,
            robot_description_node=self.declare_parameter(
                "robot_description_node", "robot_state_publisher"
            ).value,
            load_meshes=bool(self.declare_parameter("load_meshes", True).value),
        )
        joint_names = self.declare_parameter("joint_names", "").value
        self._node_config = PyrokiOnlineMpcNodeConfig(
            pose_topic=self.declare_parameter(
                "pose_topic", "/teleop/pose_commands"
            ).value,
            source_name=self.declare_parameter("source_name", "pyroki_mpc").value,
            source_namespace=self.declare_parameter(
                "source_namespace", "/action_sources"
            ).value,
            command_sink_mode=self.declare_parameter(
                "command_sink_mode", "em"
            ).value,
            direct_command_topic=self.declare_parameter(
                "direct_command_topic", ""
            ).value,
            step_rate_hz=float(self.declare_parameter("step_rate_hz", 50.0).value),
            max_state_age_s=float(
                self.declare_parameter("max_state_age_s", 0.05).value
            ),
            target_stale_timeout_s=float(
                self.declare_parameter("target_stale_timeout_s", 0.1).value
            ),
            joint_names=tuple(_parse_name_list(joint_names)),
        )
        self._solver_config = PyrokiOnlineMpcSolverConfig(
            horizon_steps=int(self.declare_parameter("horizon_steps", 10).value),
        )

    def _load_robot_model(self) -> None:
        robot_description = resolve_robot_description_xml(
            self,
            source_node=self._robot_config.robot_description_node,
        )
        self._robot = load_robot_from_urdf(
            robot_description,
            load_meshes=self._robot_config.load_meshes,
        )
        self._robot_collision = load_robot_collision_from_urdf(
            robot_description,
            load_meshes=True,
        )
        self._joint_names = list(self._node_config.joint_names)
        if not self._joint_names:
            self._joint_names = list(self._robot.joints.actuated_names)
        self.get_logger().info(
            f"PyRoki MPC model actuated joints ({len(self._joint_names)}): "
            f"{self._joint_names}"
        )

    def _configure_planner(self) -> None:
        backend = PyrokiHorizonMpcBackend(
            self._robot,
            self._robot_config.target_link_name,
            robot_collision=self._robot_collision,
            solver_config=self._solver_config,
        )
        self.configure(
            backend=backend,
            joint_names=self._joint_names,
            config=OnlineMpcSourceConfig(
                source_name=self._node_config.source_name,
                source_namespace=self._node_config.source_namespace,
                step_rate_hz=self._node_config.step_rate_hz,
                max_state_age_s=self._node_config.max_state_age_s,
                max_target_age_s=self._node_config.target_stale_timeout_s,
                command_sink_mode=self._node_config.command_sink_mode,
                direct_command_topic=self._node_config.direct_command_topic or None,
            ),
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        position = msg.pose.position
        orientation = msg.pose.orientation
        target = PoseTarget(
            position_xyz=(position.x, position.y, position.z),
            orientation_wxyz=(
                orientation.w,
                orientation.x,
                orientation.y,
                orientation.z,
            ),
        )
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp_s <= 0.0:
            stamp_s = self.get_clock().now().nanoseconds / 1e9
        self.set_target(target, stamp_s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PyrokiOnlineMpcPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
