"""PyRoki global setpoint planner ROS node for Marvin / Piper teleop."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64

from manipulation_motion_planning.planner_sources import (
    GlobalSetpointPlannerSource,
    GlobalSetpointSourceConfig,
)
from manipulation_motion_planning.robot_description import resolve_robot_description_xml

from . import _bootstrap  # noqa: F401
from .config import PyrokiGlobalSetpointNodeConfig, PyrokiRobotLoadConfig
from .pyroki_setpoint_backend import JparseSolverConfig, PyrokiJparseSetpointBackend
from .robot_loader import load_robot_from_urdf


def _parse_name_list(value: str) -> list[str]:
    return [part.strip() for part in value.split(",") if part.strip()]


class PyrokiSetpointPlannerNode(GlobalSetpointPlannerSource):
    """Cartesian pose -> J-PARSE setpoint IK -> EM `joint_target`."""

    def __init__(self) -> None:
        super().__init__("pyroki_global_setpoint_planner")
        self._load_parameters()
        self._load_robot_model()
        self._configure_planner()
        self._create_diagnostics_publishers()

    def _load_parameters(self) -> None:
        self._robot_config = PyrokiRobotLoadConfig(
            target_link_name=self.declare_parameter(
                "target_link_name", "flange_L"
            ).value,
            robot_description_node=self.declare_parameter(
                "robot_description_node", "robot_state_publisher"
            ).value,
            load_meshes=bool(self.declare_parameter("load_meshes", False).value),
        )
        output_joint_names = self.declare_parameter("output_joint_names", "").value
        self._node_config = PyrokiGlobalSetpointNodeConfig(
            pose_topic=self.declare_parameter(
                "pose_topic", "/teleop/pose_commands"
            ).value,
            source_name=self.declare_parameter("source_name", "joint_slider").value,
            source_namespace=self.declare_parameter(
                "source_namespace", "/action_sources"
            ).value,
            command_sink_mode=self.declare_parameter(
                "command_sink_mode", "em"
            ).value,
            direct_command_topic=self.declare_parameter(
                "direct_command_topic", ""
            ).value,
            plan_rate_hz=float(self.declare_parameter("plan_rate_hz", 50.0).value),
            max_state_age_s=float(
                self.declare_parameter("max_state_age_s", 0.05).value
            ),
            pose_stale_timeout_s=float(
                self.declare_parameter("pose_stale_timeout_s", 0.5).value
            ),
            output_joint_names=tuple(_parse_name_list(output_joint_names)),
        )

        self._solver = JparseSolverConfig(
            method=self.declare_parameter("method", "jparse").value,
            gamma=float(self.declare_parameter("gamma", 0.3).value),
            singular_direction_gain_position=float(
                self.declare_parameter("singular_direction_gain_position", 1.0).value
            ),
            singular_direction_gain_angular=float(
                self.declare_parameter("singular_direction_gain_angular", 1.0).value
            ),
            position_gain=float(self.declare_parameter("position_gain", 5.0).value),
            orientation_gain=float(
                self.declare_parameter("orientation_gain", 1.0).value
            ),
            nullspace_gain=float(self.declare_parameter("nullspace_gain", 0.1).value),
            max_joint_velocity=float(
                self.declare_parameter("max_joint_velocity", 3.0).value
            ),
            dls_damping=float(self.declare_parameter("dls_damping", 0.05).value),
        )
        self._max_iterations = int(
            self.declare_parameter("max_iterations_per_tick", 1).value
        )
        self._max_step_rad = float(
            self.declare_parameter("max_step_rad", 0.05).value
        )
        self._position_tolerance_m = float(
            self.declare_parameter("position_tolerance_m", 1e-3).value
        )
        self._orientation_tolerance_rad = float(
            self.declare_parameter("orientation_tolerance_rad", 1e-2).value
        )
        self._singularity_warn_threshold = float(
            self.declare_parameter("singularity_warn_threshold", 0.03).value
        )
        self._singularity_warn_throttle_s = float(
            self.declare_parameter("singularity_warn_throttle_s", 5.0).value
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
        self._state_joint_names = list(self._robot.joints.actuated_names)
        self._output_joint_names = list(self._node_config.output_joint_names)
        if not self._output_joint_names:
            self._output_joint_names = list(self._state_joint_names)
        self.get_logger().info(
            f"PyRoki model actuated joints ({len(self._state_joint_names)}): "
            f"{self._state_joint_names}"
        )

    def _configure_planner(self) -> None:
        backend = PyrokiJparseSetpointBackend(
            self._robot,
            self._robot_config.target_link_name,
            solver=self._solver,
        )
        self.configure(
            backend=backend,
            state_joint_names=self._state_joint_names,
            output_joint_names=self._output_joint_names,
            config=GlobalSetpointSourceConfig(
                source_name=self._node_config.source_name,
                source_namespace=self._node_config.source_namespace,
                pose_topic=self._node_config.pose_topic,
                plan_rate_hz=self._node_config.plan_rate_hz,
                max_state_age_s=self._node_config.max_state_age_s,
                max_target_age_s=self._node_config.pose_stale_timeout_s,
                command_sink_mode=self._node_config.command_sink_mode,
                direct_command_topic=self._node_config.direct_command_topic or None,
                plan_options={
                    "max_iterations": self._max_iterations,
                    "max_step_rad": self._max_step_rad,
                    "position_tolerance_m": self._position_tolerance_m,
                    "orientation_tolerance_rad": self._orientation_tolerance_rad,
                },
            ),
        )

    def _create_diagnostics_publishers(self) -> None:
        self._manipulability_pub = self.create_publisher(
            Float64, "~/manipulability", 10
        )
        self._inverse_condition_pub = self.create_publisher(
            Float64, "~/inverse_condition_number", 10
        )

    def _note_success(self, diagnostics: dict) -> None:
        super()._note_success(diagnostics)
        if not diagnostics:
            return
        if "manipulability" in diagnostics:
            msg = Float64()
            msg.data = float(diagnostics["manipulability"])
            self._manipulability_pub.publish(msg)
        if "inverse_condition_number" in diagnostics:
            msg = Float64()
            msg.data = float(diagnostics["inverse_condition_number"])
            self._inverse_condition_pub.publish(msg)
        if (
            self._solver.method == "jparse"
            and float(diagnostics.get("inverse_condition_number", 1.0))
            < self._singularity_warn_threshold
        ):
            self.get_logger().warn(
                "Near singular configuration: "
                f"manipulability={diagnostics.get('manipulability')}, "
                f"inverse_condition_number={diagnostics.get('inverse_condition_number')}, "
                f"gamma={self._solver.gamma:.3f}",
                throttle_duration_sec=self._singularity_warn_throttle_s,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PyrokiSetpointPlannerNode()
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
