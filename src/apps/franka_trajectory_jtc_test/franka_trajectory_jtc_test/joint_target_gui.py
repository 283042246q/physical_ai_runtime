#!/usr/bin/env python3
"""FR3 joint-target GUI / auto-send → EM → effort JTC or impedance JSPC.

Distributed (RT + operator):

  # RT / robot host (JTC default; override yaml/controller for JSPC)
  ros2 launch franka_trajectory_jtc_test joint_gui_rt_bringup.launch.py \\
    use_fake_hardware:=false robot_ip:=192.168.2.101

  # Operator PC
  ros2 launch franka_trajectory_jtc_test joint_gui_operator.launch.py
"""

from __future__ import annotations

import math
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

FR3_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
JOINT_LIMITS_LOWER = [-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508]
JOINT_LIMITS_UPPER = [2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508]

REFERENCE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

AUTO_SEND_STREAM = "stream"
AUTO_SEND_SINE = "sine"
OUTPUT_JOINT_TARGET = "joint_target"
OUTPUT_JOINT_TRAJECTORY_GOAL = "joint_trajectory_goal"


def _clamp_positions(positions: list[float]) -> list[float]:
    return [
        min(max(value, lower), upper)
        for value, lower, upper in zip(
            positions, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER
        )
    ]


class JointTargetBridge(Node):
    """ROS side: read joint states, publish EM setpoints (JTC or JSPC)."""

    def __init__(self) -> None:
        super().__init__("joint_target_gui")
        self.declare_parameter("joint_state_topic", "/franka/joint_states")
        self.declare_parameter(
            "output_contract", OUTPUT_JOINT_TRAJECTORY_GOAL
        )
        self.declare_parameter(
            "goal_topic", "/action_sources/joint_gui/joint_trajectory_goal"
        )
        self.declare_parameter("move_duration_s", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("use_gui", True)
        self.declare_parameter("sync_on_start", True)
        self.declare_parameter("auto_send", False)
        self.declare_parameter("auto_send_rate_hz", 10.0)
        self.declare_parameter("auto_send_mode", AUTO_SEND_STREAM)
        self.declare_parameter("auto_send_joint_index", 3)
        self.declare_parameter("auto_send_amplitude_rad", 0.10)
        self.declare_parameter("auto_send_period_s", 4.0)

        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._output_contract = str(
            self.get_parameter("output_contract").value
        ).strip().lower()
        self._goal_topic = str(self.get_parameter("goal_topic").value)
        self._move_duration_s = max(
            float(self.get_parameter("move_duration_s").value), 0.05
        )
        self._min_publish_interval_s = 1.0 / float(
            self.get_parameter("publish_rate_hz").value
        )
        self.use_gui = bool(self.get_parameter("use_gui").value)
        self.sync_on_start = bool(self.get_parameter("sync_on_start").value)
        self._auto_send_mode = str(self.get_parameter("auto_send_mode").value).strip().lower()
        self._auto_send_joint_index = int(
            self.get_parameter("auto_send_joint_index").value
        )
        self._auto_send_amplitude_rad = float(
            self.get_parameter("auto_send_amplitude_rad").value
        )
        self._auto_send_period_s = max(
            float(self.get_parameter("auto_send_period_s").value), 0.5
        )

        if self._output_contract not in {
            OUTPUT_JOINT_TARGET,
            OUTPUT_JOINT_TRAJECTORY_GOAL,
        }:
            raise ValueError(
                "output_contract must be "
                f"{OUTPUT_JOINT_TARGET!r} or {OUTPUT_JOINT_TRAJECTORY_GOAL!r}"
            )
        if self._auto_send_mode not in {AUTO_SEND_STREAM, AUTO_SEND_SINE}:
            raise ValueError(
                f"auto_send_mode must be {AUTO_SEND_STREAM!r} or {AUTO_SEND_SINE!r}"
            )
        if not 0 <= self._auto_send_joint_index < len(FR3_JOINTS):
            raise ValueError("auto_send_joint_index must be in [0, 6]")

        self._measured: list[float] | None = None
        self._stream_target: list[float] | None = None
        self._pending_target: list[float] | None = None
        self._last_publish_monotonic = 0.0
        self._auto_send_enabled = bool(self.get_parameter("auto_send").value)
        self._auto_send_start_monotonic = time.monotonic()
        self._did_startup_sync = False
        self._lock = threading.Lock()

        self._status = "waiting for /franka/joint_states"

        if self._output_contract == OUTPUT_JOINT_TARGET:
            self._goal_pub = self.create_publisher(
                JointState, self._goal_topic, REFERENCE_QOS
            )
        else:
            self._goal_pub = self.create_publisher(
                JointTrajectory, self._goal_topic, REFERENCE_QOS
            )
        self.create_subscription(
            JointState, self._joint_state_topic, self._on_joint_state, 10
        )
        self.create_timer(0.05, self._flush_pending)
        self.create_timer(0.5, self._log_status)

        auto_send_rate_hz = float(self.get_parameter("auto_send_rate_hz").value)
        if auto_send_rate_hz <= 0.0:
            raise ValueError("auto_send_rate_hz must be positive")
        self.create_timer(1.0 / auto_send_rate_hz, self._auto_send_tick)

        self.get_logger().info(
            "Joint target bridge: state=%s contract=%s topic=%s gui=%s auto_send=%s mode=%s"
            % (
                self._joint_state_topic,
                self._output_contract,
                self._goal_topic,
                self.use_gui,
                self._auto_send_enabled,
                self._auto_send_mode,
            )
        )

    def set_auto_send(self, enabled: bool) -> None:
        with self._lock:
            self._auto_send_enabled = enabled
            self._auto_send_start_monotonic = time.monotonic()
        mode = "enabled" if enabled else "disabled"
        self.get_logger().info(f"Auto-send {mode}")

    def auto_send_enabled(self) -> bool:
        with self._lock:
            return self._auto_send_enabled

    def set_stream_target(self, positions: list[float]) -> None:
        with self._lock:
            self._stream_target = _clamp_positions(list(positions))

    def request_target(self, positions: list[float]) -> None:
        with self._lock:
            self._stream_target = _clamp_positions(list(positions))
            self._pending_target = list(self._stream_target)

    def startup_sync_done(self) -> bool:
        return self._did_startup_sync

    def mark_startup_sync_done(self) -> None:
        self._did_startup_sync = True

    def measured_positions(self) -> list[float] | None:
        with self._lock:
            return None if self._measured is None else list(self._measured)

    def status_text(self) -> str:
        with self._lock:
            return self._status

    def _on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in FR3_JOINTS):
            return
        measured = [float(positions[name]) for name in FR3_JOINTS]
        with self._lock:
            self._measured = measured
            if self._stream_target is None:
                self._stream_target = list(measured)
            em_ready = self._goal_pub.get_subscription_count() > 0
            self._status = (
                "ready (joint states + EM)"
                if em_ready
                else "joint states OK; waiting for EM on RT host"
            )

    def _log_status(self) -> None:
        with self._lock:
            status = self._status
            measured = None if self._measured is None else "ok"
            em_count = self._goal_pub.get_subscription_count()
        if measured is None:
            self.get_logger().warn(
                f"{status} — no data on {self._joint_state_topic}",
                throttle_duration_sec=5.0,
            )
            return
        if em_count == 0:
            self.get_logger().warn(
                f"{status} — EM not subscribed yet on {self._goal_topic}",
                throttle_duration_sec=5.0,
            )

    def _flush_pending(self) -> None:
        now = time.monotonic()
        with self._lock:
            if self._pending_target is None:
                return
            if now - self._last_publish_monotonic < self._min_publish_interval_s:
                return
            target = list(self._pending_target)
            self._pending_target = None
            self._last_publish_monotonic = now

        self._publish_goal(target)

    def _auto_send_tick(self) -> None:
        with self._lock:
            if not self._auto_send_enabled or self._measured is None:
                return
            measured = list(self._measured)
            stream_target = (
                list(self._stream_target)
                if self._stream_target is not None
                else list(measured)
            )
            mode = self._auto_send_mode

        if mode == AUTO_SEND_SINE:
            elapsed = time.monotonic() - self._auto_send_start_monotonic
            phase = 2.0 * math.pi * elapsed / self._auto_send_period_s
            target = list(measured)
            target[self._auto_send_joint_index] += self._auto_send_amplitude_rad * math.sin(
                phase
            )
            target = _clamp_positions(target)
        else:
            target = _clamp_positions(stream_target)

        self._publish_goal(target)

    def _publish_goal(self, target: list[float]) -> None:
        if self._output_contract == OUTPUT_JOINT_TARGET:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(FR3_JOINTS)
            msg.position = list(target)
            self._goal_pub.publish(msg)
            return

        with self._lock:
            start = (
                list(self._measured)
                if self._measured is not None
                else list(target)
            )
        self._goal_pub.publish(
            self._make_trajectory(start, target, self._move_duration_s)
        )

    @staticmethod
    def _make_trajectory(
        start: list[float], target: list[float], duration_s: float
    ) -> JointTrajectory:
        duration_s = max(duration_s, 0.05)
        velocities = [
            (goal - current) / duration_s for current, goal in zip(start, target)
        ]
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.velocities = velocities
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int(round((duration_s % 1.0) * 1e9))

        msg = JointTrajectory()
        # Zero stamp: EM accepts it when reject_zero_stamped_references=false.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = list(FR3_JOINTS)
        msg.points = [point]
        return msg


class JointTargetGui:
    def __init__(self, bridge: JointTargetBridge) -> None:
        self._bridge = bridge
        self._root = tk.Tk()
        sink = (
            "JSPC"
            if bridge._output_contract == OUTPUT_JOINT_TARGET
            else "effort JTC"
        )
        self._root.title(f"FR3 Joint Target → EM → {sink}")
        self._root.geometry("560x500")

        self._values: list[tk.DoubleVar] = []
        self._scales: list[ttk.Scale] = []
        self._labels: list[ttk.Label] = []
        self._auto_send_var = tk.BooleanVar(value=bridge.auto_send_enabled())

        main = ttk.Frame(self._root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        self._status_label = ttk.Label(
            main,
            text="Status: starting…",
            wraplength=520,
        )
        self._status_label.pack(anchor=tk.W, pady=(0, 8))

        ttk.Label(
            main,
            text=(
                "Distributed: run joint_gui_rt_bringup on the RT PC first. "
                "Drag sliders or enable Auto-send to stream targets."
            ),
            wraplength=520,
        ).pack(anchor=tk.W, pady=(0, 8))

        for index, joint_name in enumerate(FR3_JOINTS):
            row = ttk.Frame(main)
            row.pack(fill=tk.X, pady=4)
            ttk.Label(row, text=joint_name, width=12).pack(side=tk.LEFT)
            value = tk.DoubleVar(value=0.0)
            scale = ttk.Scale(
                row,
                from_=JOINT_LIMITS_LOWER[index],
                to=JOINT_LIMITS_UPPER[index],
                orient=tk.HORIZONTAL,
                variable=value,
                command=lambda _val, idx=index: self._on_slider(idx),
            )
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
            label = ttk.Label(row, text="0.000", width=8)
            label.pack(side=tk.RIGHT)
            self._values.append(value)
            self._scales.append(scale)
            self._labels.append(label)

        control_row = ttk.Frame(main)
        control_row.pack(fill=tk.X, pady=(12, 0))
        ttk.Button(
            control_row, text="Sync from robot", command=self._sync_from_robot
        ).pack(side=tk.LEFT)
        ttk.Button(control_row, text="Send once", command=self._send_all).pack(
            side=tk.LEFT, padx=8
        )
        ttk.Checkbutton(
            control_row,
            text="Auto-send",
            variable=self._auto_send_var,
            command=self._on_auto_send_toggle,
        ).pack(side=tk.LEFT, padx=8)

        self._root.after(200, self._poll_measured)
        self._root.after(500, self._refresh_status)

    def _on_auto_send_toggle(self) -> None:
        self._bridge.set_auto_send(self._auto_send_var.get())

    def _on_slider(self, index: int) -> None:
        value = self._values[index].get()
        self._labels[index].configure(text=f"{value:.3f}")
        positions = [var.get() for var in self._values]
        self._bridge.set_stream_target(positions)
        if not self._bridge.auto_send_enabled():
            self._bridge.request_target(positions)

    def _send_all(self) -> None:
        self._bridge.request_target([var.get() for var in self._values])

    def _sync_from_robot(self) -> None:
        measured = self._bridge.measured_positions()
        if measured is None:
            return
        for index, value in enumerate(measured):
            clamped = min(
                max(value, JOINT_LIMITS_LOWER[index]), JOINT_LIMITS_UPPER[index]
            )
            self._values[index].set(clamped)
            self._labels[index].configure(text=f"{clamped:.3f}")
        self._bridge.set_stream_target([var.get() for var in self._values])

    def _poll_measured(self) -> None:
        if (
            self._bridge.sync_on_start
            and not self._bridge.startup_sync_done()
            and self._bridge.measured_positions() is not None
        ):
            self._sync_from_robot()
            self._bridge.mark_startup_sync_done()
        elif all(math.isclose(var.get(), 0.0, abs_tol=1e-6) for var in self._values):
            self._sync_from_robot()
        self._root.after(500, self._poll_measured)

    def _refresh_status(self) -> None:
        self._status_label.configure(text=f"Status: {self._bridge.status_text()}")
        self._root.after(500, self._refresh_status)

    def run(self) -> None:
        self._root.mainloop()


def _spin_ros(bridge: JointTargetBridge) -> None:
    try:
        rclpy.spin(bridge)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def main() -> None:
    rclpy.init()
    bridge = JointTargetBridge()
    spinner = threading.Thread(target=_spin_ros, args=(bridge,), daemon=True)
    spinner.start()

    try:
        if bridge.use_gui:
            JointTargetGui(bridge).run()
        else:
            bridge.get_logger().info(
                "Headless auto-send running; Ctrl+C to stop."
            )
            while rclpy.ok():
                time.sleep(0.2)
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
