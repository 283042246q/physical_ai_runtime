"""cuRobo online MPC planner ROS node."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException

from manipulation_motion_planning.contracts import PoseTarget
from manipulation_motion_planning.planner_sources import (
    OnlineMpcPlannerSource,
    OnlineMpcSourceConfig,
)

from .config import (
    CuroboMpcSolverConfig,
    CuroboOnlineMpcNodeConfig,
    CuroboRobotConfig,
)
from .curobo_backend import CuroboMpcBackend


def _parse_name_list(value: str) -> list[str]:
    return [part.strip() for part in value.split(",") if part.strip()]


class CuroboOnlineMpcPlannerNode(OnlineMpcPlannerSource):
    """Pose target stream -> cuRobo MPC -> EM `joint_chunk`."""

    def __init__(self) -> None:
        super().__init__("curobo_online_mpc_planner")
        self._load_parameters()
        self._configure_planner()
        self.create_subscription(PoseStamped, self._node_config.pose_topic, self._on_pose, 10)

    def _load_parameters(self) -> None:
        joint_names = self.declare_parameter("joint_names", "").value
        scene_model = self.declare_parameter("scene_model", "").value
        self._robot_config = CuroboRobotConfig(
            robot=self.declare_parameter("robot", "franka.yml").value,
            scene_model=scene_model or None,
            target_link_name=self.declare_parameter("target_link_name", "").value,
            use_cuda_graph=bool(self.declare_parameter("use_cuda_graph", True).value),
            self_collision_check=bool(
                self.declare_parameter("self_collision_check", True).value
            ),
            load_collision_spheres=bool(
                self.declare_parameter("load_collision_spheres", True).value
            ),
        )
        self._node_config = CuroboOnlineMpcNodeConfig(
            pose_topic=self.declare_parameter(
                "pose_topic", "/teleop/pose_commands"
            ).value,
            source_name=self.declare_parameter("source_name", "curobo_mpc").value,
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
        self._solver_config = CuroboMpcSolverConfig(
            optimization_dt=float(
                self.declare_parameter("optimization_dt", 0.02).value
            ),
            interpolation_steps=int(
                self.declare_parameter("interpolation_steps", 4).value
            ),
            horizon_points=int(self.declare_parameter("horizon_points", 1).value),
            position_tolerance_m=float(
                self.declare_parameter("position_tolerance_m", 0.005).value
            ),
            orientation_tolerance_rad=float(
                self.declare_parameter("orientation_tolerance_rad", 0.05).value
            ),
            optimizer_collision_activation_distance_m=float(
                self.declare_parameter(
                    "optimizer_collision_activation_distance_m", 0.01
                ).value
            ),
            warm_start_optimization_num_iters=int(
                self.declare_parameter("warm_start_optimization_num_iters", 200).value
            ),
            cold_start_optimization_num_iters=int(
                self.declare_parameter("cold_start_optimization_num_iters", 300).value
            ),
        )

    def _configure_planner(self) -> None:
        backend = CuroboMpcBackend(
            robot_config=self._robot_config,
            solver_config=self._solver_config,
        )
        joint_names = list(self._node_config.joint_names) or backend.joint_names
        self.get_logger().info(
            f"cuRobo MPC joints ({len(joint_names)}): {joint_names}; "
            f"tool_frames={backend.tool_frames}"
        )
        self.configure(
            backend=backend,
            joint_names=joint_names,
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
    node = CuroboOnlineMpcPlannerNode()
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
