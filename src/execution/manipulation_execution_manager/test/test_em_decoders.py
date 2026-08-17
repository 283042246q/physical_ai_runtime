#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
#
# Unit tests for EM decoders, arbitration, and state-machine guards.
#
# Run:
#   pytest test/test_em_decoders.py
#   # or via colcon test in a ROS 2 workspace

from __future__ import annotations

import math
import pytest
import rclpy

# One-time rclpy init for the whole test run (idempotent).
if not rclpy.ok():
    rclpy.init()

# Module under test — importing is safe (no side effects; main() is opt-in).
from manipulation_execution_manager import execution_manager_node as em
from manipulation_execution_manager import execution_manager_arbitration as arb
from manipulation_execution_manager import execution_manager_ingest as ingest
from manipulation_execution_manager import execution_manager_status as status_mod
from manipulation_execution_manager import execution_manager_validation as validation


@pytest.mark.parametrize(
    ("raw", "message"),
    [
        ("[]", "must decode to an object"),
        ('{"teleop": 1}', "sources.teleop must be a JSON object"),
        ('{"teleop": {"priority": true}}', "priority must be an integer"),
        (
            '{"teleop": {"inactive_timeout_s": 0}}',
            "inactive_timeout_s must be finite and positive",
        ),
    ],
)
def test_sources_config_rejects_invalid_contract(raw, message):
    with pytest.raises(RuntimeError, match=message):
        em.parse_sources_config(raw)


def test_sources_config_accepts_json_object():
    parsed = em.parse_sources_config(
        '{"teleop": {"priority": 100, "pose_contracts": ["pose_target"]}}'
    )
    assert parsed["teleop"]["priority"] == 100


# ---------------------------------------------------------------------------
# Shared test helpers
# ---------------------------------------------------------------------------

def _joint_names(n: int = 3) -> list[str]:
    return [f"joint{i + 1}" for i in range(n)]


def _make_joint_state(
    names: list[str],
    positions: list[float],
    sec: int = 100,
    nanosec: int = 0,
):
    """Return a minimal JointState for decode_joint_target tests."""
    from sensor_msgs.msg import JointState
    msg = JointState()
    msg.name = list(names)
    msg.position = [float(p) for p in positions]
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    return msg


def _make_joint_trajectory(
    joint_names: list[str],
    points_s: list[tuple[float, list[float], list[float] | None]],
    *,
    sec: int = 100,
    nanosec: int = 0,
):
    """Return a JointTrajectory message.

    Each element in *points_s* is (time_from_start_s, positions, velocities|None).
    """
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    msg = JointTrajectory()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.joint_names = list(joint_names)
    for t_s, positions, velocities in points_s:
        pt = JointTrajectoryPoint()
        pt.time_from_start.sec = int(t_s)
        pt.time_from_start.nanosec = int((t_s - int(t_s)) * 1e9)
        pt.positions = [float(p) for p in positions]
        if velocities is not None:
            pt.velocities = [float(v) for v in velocities]
        msg.points.append(pt)
    return msg


# ---------------------------------------------------------------------------
# decode_joint_target
# ---------------------------------------------------------------------------

class TestDecodeJointTarget:
    """JointState → single-point JointTrajectory (reorder, clamp, time validation)."""

    @staticmethod
    def _clock():
        return rclpy.clock.Clock()

    def test_valid_reordered(self):
        msg = _make_joint_state(["joint3", "joint1", "joint2"], [0.3, 0.1, 0.2])
        out = em.decode_joint_target(
            msg, _joint_names(3), [], [], reject_out_of_bounds=False,
            clock=self._clock(), reject_zero_stamp=False,
        )
        assert out is not None
        assert out.joint_names == _joint_names(3)                       # always expected order
        assert list(out.points[0].positions) == [0.1, 0.2, 0.3]        # reordered

    def test_missing_joint_returns_none(self):
        msg = _make_joint_state(["joint1"], [0.1])
        out = em.decode_joint_target(
            msg, _joint_names(3), [], [], reject_out_of_bounds=False,
            clock=self._clock(), reject_zero_stamp=False,
        )
        assert out is None

    def test_nonfinite_position_returns_none(self):
        msg = _make_joint_state(_joint_names(3), [0.1, float("nan"), 0.3])
        out = em.decode_joint_target(
            msg, _joint_names(3), [], [], reject_out_of_bounds=False,
            clock=self._clock(), reject_zero_stamp=False,
        )
        assert out is None

    def test_zero_stamp_rejected_when_enabled(self):
        msg = _make_joint_state(_joint_names(2), [0.0, 0.0], sec=0, nanosec=0)
        out = em.decode_joint_target(
            msg, _joint_names(2), [], [], reject_out_of_bounds=False,
            clock=self._clock(), reject_zero_stamp=True,
        )
        assert out is None

    def test_zero_stamp_fallback_when_disabled(self):
        msg = _make_joint_state(_joint_names(2), [0.0, 0.0], sec=0, nanosec=0)
        out = em.decode_joint_target(
            msg, _joint_names(2), [], [], reject_out_of_bounds=False,
            clock=self._clock(), reject_zero_stamp=False,
        )
        assert out is not None
        # Clock fallback should produce a non-zero stamp.
        assert out.header.stamp.sec > 0 or out.header.stamp.nanosec > 0

    def test_out_of_bounds_clamped(self):
        msg = _make_joint_state(_joint_names(2), [99.0, -99.0])
        out = em.decode_joint_target(
            msg, _joint_names(2),
            joint_limits_lower=[-5.0, -5.0],
            joint_limits_upper=[5.0, 5.0],
            reject_out_of_bounds=False,
            clock=self._clock(), reject_zero_stamp=False,
        )
        assert out is not None
        assert list(out.points[0].positions) == [5.0, -5.0]  # clamped

    def test_out_of_bounds_rejected(self):
        msg = _make_joint_state(_joint_names(2), [99.0, 0.0])
        out = em.decode_joint_target(
            msg, _joint_names(2),
            joint_limits_lower=[-5.0, -5.0],
            joint_limits_upper=[5.0, 5.0],
            reject_out_of_bounds=True,
            clock=self._clock(), reject_zero_stamp=False,
        )
        assert out is None

    def test_single_point_output(self):
        msg = _make_joint_state(_joint_names(3), [0.1, 0.2, 0.3])
        out = em.decode_joint_target(
            msg, _joint_names(3), [], [], reject_out_of_bounds=False,
            clock=self._clock(), reject_zero_stamp=False,
        )
        assert out is not None
        assert len(out.points) == 1
        assert out.points[0].time_from_start.sec == 0
        assert out.points[0].time_from_start.nanosec == 0

    def test_empty_limits_ok(self):
        msg = _make_joint_state(_joint_names(3), [-999.0, 0.0, 999.0])
        out = em.decode_joint_target(
            msg, _joint_names(3), [], [], reject_out_of_bounds=False,
            clock=self._clock(), reject_zero_stamp=False,
        )
        assert out is not None


# ---------------------------------------------------------------------------
# decode_joint_chunk
# ---------------------------------------------------------------------------

class TestDecodeJointChunk:
    """JointTrajectory passthrough: reorder + validate."""

    def test_empty_joint_names_returns_none(self):
        msg = _make_joint_trajectory([], [(0.0, [0.1], None)])
        out = em.decode_joint_chunk(msg, _joint_names(1), [], [], False)
        assert out is None

    def test_empty_points_returns_none(self):
        msg = _make_joint_trajectory(_joint_names(2), [])
        out = em.decode_joint_chunk(msg, _joint_names(2), [], [], False)
        assert out is None

    def test_missing_joint_returns_none(self):
        msg = _make_joint_trajectory(["joint1"], [(0.0, [0.1], None)])
        out = em.decode_joint_chunk(msg, _joint_names(3), [], [], False)
        assert out is None

    def test_reorder_identity(self):
        msg = _make_joint_trajectory(
            _joint_names(3),
            [(0.0, [0.1, 0.2, 0.3], None),
             (1.0, [1.1, 1.2, 1.3], None)],
        )
        out = em.decode_joint_chunk(msg, _joint_names(3), [], [], False)
        assert out is not None
        assert out.joint_names == _joint_names(3)
        assert list(out.points[0].positions) == [0.1, 0.2, 0.3]
        assert list(out.points[1].positions) == [1.1, 1.2, 1.3]

    def test_reorder_scrambled(self):
        msg = _make_joint_trajectory(
            ["joint3", "joint1", "joint2"],
            [(0.0, [0.3, 0.1, 0.2], None)],
        )
        out = em.decode_joint_chunk(msg, _joint_names(3), [], [], False)
        assert out is not None
        assert list(out.points[0].positions) == [0.1, 0.2, 0.3]

    def test_velocities_reordered(self):
        msg = _make_joint_trajectory(
            ["joint2", "joint1"],
            [(0.0, [10.0, 20.0], [0.5, 1.5])],
        )
        out = em.decode_joint_chunk(msg, ["joint1", "joint2"], [], [], False)
        assert out is not None
        assert list(out.points[0].positions) == [20.0, 10.0]   # joint1, joint2
        assert list(out.points[0].velocities) == [1.5, 0.5]    # reordered too

    def test_out_of_bounds_clamped(self):
        msg = _make_joint_trajectory(
            _joint_names(2),
            [(0.0, [99.0, -99.0], None)],
        )
        out = em.decode_joint_chunk(
            msg, _joint_names(2), [-5.0, -5.0], [5.0, 5.0], reject_out_of_bounds=False,
        )
        assert out is not None
        assert list(out.points[0].positions) == [5.0, -5.0]

    def test_out_of_bounds_rejected(self):
        msg = _make_joint_trajectory(
            _joint_names(2),
            [(0.0, [99.0, 0.0], None)],
        )
        out = em.decode_joint_chunk(
            msg, _joint_names(2), [-5.0, -5.0], [5.0, 5.0], reject_out_of_bounds=True,
        )
        assert out is None

    def test_nonfinite_position_returns_none(self):
        msg = _make_joint_trajectory(
            _joint_names(3),
            [(0.0, [0.1, float("inf"), 0.3], None)],
        )
        out = em.decode_joint_chunk(msg, _joint_names(3), [], [], False)
        assert out is None

    def test_too_few_positions_returns_none(self):
        msg = _make_joint_trajectory(
            _joint_names(3),
            [(0.0, [0.1, 0.2], None)],  # only 2 positions for 3 joints
        )
        out = em.decode_joint_chunk(msg, _joint_names(3), [], [], False)
        assert out is None


# ---------------------------------------------------------------------------
# decode_pose_target / decode_pose_chunk / decode_delta_pose / decode_delta_chunk
# ---------------------------------------------------------------------------

class TestDecodePoseTarget:
    def test_valid_passthrough(self):
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.header.stamp.sec = 100
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.position.z = 3.0
        msg.pose.orientation.w = 1.0
        out = em.decode_pose_target(msg)
        assert out is not None
        assert out.pose.position.x == 1.0

    def test_invalid_pose_returns_none(self):
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.pose.position.x = float("nan")
        msg.pose.orientation.w = 1.0
        out = em.decode_pose_target(msg)
        assert out is None


class TestDecodePoseChunk:
    def test_valid_passthrough(self):
        from geometry_msgs.msg import Pose, PoseArray
        p = Pose(); p.position.x = 1.0; p.orientation.w = 1.0
        msg = PoseArray()
        msg.poses = [p, p]
        out = em.decode_pose_chunk(msg)
        assert out is not None
        assert len(out.poses) == 2

    def test_empty_returns_none(self):
        from geometry_msgs.msg import PoseArray
        assert em.decode_pose_chunk(PoseArray()) is None


class TestDecodeDeltaPose:
    def test_delta_to_absolute(self):
        from geometry_msgs.msg import PoseStamped
        ref = em.pose_utils.array_to_pose(
            em.pose_utils.np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
        rp = em.pose_utils.np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        delta = PoseStamped()
        delta.header.stamp.sec = 100
        delta.pose.position.x = 0.1
        delta.pose.position.y = 0.0
        delta.pose.position.z = 0.0
        delta.pose.orientation.w = 1.0
        out = em.decode_delta_pose(delta, rp)
        assert out is not None
        assert abs(out.pose.position.x - 0.6) < 1e-9

    def test_none_ref_returns_none(self):
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.pose.orientation.w = 1.0
        assert em.decode_delta_pose(msg, None) is None


class TestDecodeDeltaChunk:
    def test_chained_deltas(self):
        from geometry_msgs.msg import Pose, PoseArray
        ref = em.pose_utils.np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        p1 = Pose(); p1.position.x = 0.1; p1.orientation.w = 1.0
        p2 = Pose(); p2.position.y = 0.2; p2.orientation.w = 1.0
        msg = PoseArray()
        msg.header.stamp.sec = 100
        msg.poses = [p1, p2]
        out = em.decode_delta_chunk(msg, ref)
        assert out is not None
        assert len(out.poses) == 2
        # First delta applied
        assert abs(out.poses[0].position.x - 0.1) < 1e-9
        # Second delta chained: x stays 0.1, y becomes 0.2
        assert abs(out.poses[1].position.x - 0.1) < 1e-9
        assert abs(out.poses[1].position.y - 0.2) < 1e-9

    def test_none_ref_returns_none(self):
        from geometry_msgs.msg import Pose, PoseArray
        p = Pose(); p.orientation.w = 1.0
        msg = PoseArray(); msg.poses = [p]
        assert em.decode_delta_chunk(msg, None) is None


# ===================================================================
# Arbitration and state-machine guard tests
# ===================================================================
# These exercise the _select_winning_source / _should_switch_to logic
# directly against an actual rclpy node (the state machine lives on
# the node, not in standalone functions).


@pytest.fixture
def em_node():
    """Return a bare ExecutionManager node with test-only joints + sources."""
    # We'll set the internal state directly in each test.
    node = em.ExecutionManager.__new__(em.ExecutionManager)
    # Manually init just the Node part (no parameter server read).
    rclpy.node.Node.__init__(node, "test_em")

    node._joint_names = ["joint1", "joint2", "joint3"]
    node._stale_timeout_s = 0.5
    node._min_hold_duration_s = 0.05
    node._sources = {}
    node._winning_source = None
    node._hold_until = None
    yield node
    node.destroy_node()


def _add_source(
    node,
    name: str,
    priority: int,
    inactive_timeout_s: float = 0.3,
):
    """Register a source and return its SourceState."""
    src = em.SourceState(
        name=name, priority=priority, inactive_timeout_s=inactive_timeout_s,
    )
    node._sources[name] = src
    return src


def _stamp_source(src, node, seconds_ago: float = 0.0):
    """Set last_received_stamp and latest_reference so the source appears fresh."""
    src.last_received_stamp = node.get_clock().now() - rclpy.duration.Duration(
        seconds=seconds_ago)
    from trajectory_msgs.msg import JointTrajectory
    src.latest_reference = JointTrajectory()


def _fresh_source(name: str, priority: int, now, seconds_ago: float = 0.0):
    """Return a SourceState suitable for direct arbitration helper tests."""
    from trajectory_msgs.msg import JointTrajectory
    src = em.SourceState(name=name, priority=priority, inactive_timeout_s=0.3)
    src.last_received_stamp = now - rclpy.duration.Duration(seconds=seconds_ago)
    src.latest_reference = JointTrajectory()
    return src


class TestArbitrationHelpers:
    def test_select_winning_source_prefers_priority(self):
        now = rclpy.clock.Clock().now()
        sources = {
            "planner": _fresh_source("planner", priority=5, now=now),
            "teleop": _fresh_source("teleop", priority=10, now=now),
        }
        assert arb.select_winning_source(sources, now, stale_timeout_s=0.5) == "teleop"

    def test_select_winning_source_excludes_stale(self):
        now = rclpy.clock.Clock().now()
        sources = {"teleop": _fresh_source("teleop", priority=10, now=now, seconds_ago=1.0)}
        assert arb.select_winning_source(sources, now, stale_timeout_s=0.5) is None

    def test_should_switch_blocks_same_priority_during_hold(self):
        now = rclpy.clock.Clock().now()
        hold_until = now + rclpy.duration.Duration(seconds=1.0)
        sources = {
            "a": _fresh_source("a", priority=5, now=now),
            "b": _fresh_source("b", priority=5, now=now),
        }
        assert arb.should_switch_to("b", "a", sources, hold_until, now) is False

    def test_should_switch_allows_higher_priority_during_hold(self):
        now = rclpy.clock.Clock().now()
        hold_until = now + rclpy.duration.Duration(seconds=1.0)
        sources = {
            "planner": _fresh_source("planner", priority=5, now=now),
            "teleop": _fresh_source("teleop", priority=10, now=now),
        }
        assert arb.should_switch_to("teleop", "planner", sources, hold_until, now) is True


class TestPathSelectors:
    """Pose/twist selectors share the joint eligibility policy (priority,
    per-source inactivity, global stale, recency tie-break)."""

    @staticmethod
    def _pose_source(name, priority, now, *, chunk=False, seconds_ago=0.0,
                     inactive_timeout_s=0.3):
        from geometry_msgs.msg import PoseArray, PoseStamped
        src = em.SourceState(
            name=name, priority=priority, inactive_timeout_s=inactive_timeout_s)
        src.last_received_stamp = now - rclpy.duration.Duration(seconds=seconds_ago)
        if chunk:
            src.latest_pose_chunk = PoseArray()
        else:
            src.latest_pose_target = PoseStamped()
        return src

    @staticmethod
    def _twist_source(name, priority, now, *, seconds_ago=0.0,
                      inactive_timeout_s=0.3):
        from geometry_msgs.msg import TwistStamped
        src = em.SourceState(
            name=name, priority=priority, inactive_timeout_s=inactive_timeout_s)
        src.last_received_stamp = now - rclpy.duration.Duration(seconds=seconds_ago)
        src.latest_twist_target = TwistStamped()
        return src

    def test_pose_selector_prefers_priority(self):
        now = rclpy.clock.Clock().now()
        sources = {
            "planner": self._pose_source("planner", 5, now),
            "teleop": self._pose_source("teleop", 10, now, chunk=True),
        }
        assert arb.select_winning_pose_source(sources, now, 0.5) == "teleop"

    def test_pose_selector_respects_inactive_timeout(self):
        # Within global stale window but past the per-source inactivity timeout.
        now = rclpy.clock.Clock().now()
        sources = {
            "teleop": self._pose_source(
                "teleop", 10, now, seconds_ago=0.2, inactive_timeout_s=0.1),
        }
        assert arb.select_winning_pose_source(sources, now, 0.5) is None

    def test_pose_selector_ignores_sources_without_pose(self):
        now = rclpy.clock.Clock().now()
        sources = {"joints": _fresh_source("joints", 10, now=now)}  # joint-only
        assert arb.select_winning_pose_source(sources, now, 0.5) is None

    def test_twist_selector_prefers_priority(self):
        now = rclpy.clock.Clock().now()
        sources = {
            "planner": self._twist_source("planner", 5, now),
            "teleop": self._twist_source("teleop", 10, now),
        }
        assert arb.select_winning_twist_source(sources, now, 0.5) == "teleop"

    def test_twist_selector_excludes_stale(self):
        now = rclpy.clock.Clock().now()
        sources = {
            "teleop": self._twist_source("teleop", 10, now, seconds_ago=1.0),
        }
        assert arb.select_winning_twist_source(sources, now, 0.5) is None


class TestActiveStreamingRoute:
    """JSPC/TSKPC mutual exclusion: one global streaming winner, one route."""

    def test_higher_priority_joint_wins_route_jspc(self):
        now = rclpy.clock.Clock().now()
        joint = _fresh_source("planner", priority=10, now=now)
        joint.latest_stream_route = em.ROUTE_JSPC
        pose = TestPathSelectors._pose_source("teleop", priority=5, now=now)
        pose.latest_stream_route = em.ROUTE_TSKPC
        sources = {"planner": joint, "teleop": pose}

        winner = arb.select_active_streaming_source(sources, now, 0.5)
        assert winner == "planner"
        assert sources[winner].latest_stream_route == em.ROUTE_JSPC

    def test_higher_priority_pose_wins_route_tskpc(self):
        now = rclpy.clock.Clock().now()
        joint = _fresh_source("planner", priority=5, now=now)
        joint.latest_stream_route = em.ROUTE_JSPC
        pose = TestPathSelectors._pose_source("teleop", priority=10, now=now)
        pose.latest_stream_route = em.ROUTE_TSKPC
        sources = {"planner": joint, "teleop": pose}

        winner = arb.select_active_streaming_source(sources, now, 0.5)
        assert winner == "teleop"
        assert sources[winner].latest_stream_route == em.ROUTE_TSKPC

    def test_jtc_goal_source_excluded_from_streaming(self):
        # A JTC-only source sets last_received_stamp but no streaming reference
        # and no route, so it must not win the streaming arbiter.
        now = rclpy.clock.Clock().now()
        jtc = em.SourceState(name="planner", priority=99, inactive_timeout_s=0.3)
        jtc.last_received_stamp = now
        # latest_reference / pose / twist all None; latest_stream_route None
        assert arb.select_active_streaming_source({"planner": jtc}, now, 0.5) is None


class _FakePub:
    """Records published messages in place of a real ROS publisher."""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class TestRouteSuppression:
    """Node-level JSPC/TSKPC mutual exclusion: only the winning route emits."""

    def _add_joint(self, node, name, priority):
        from trajectory_msgs.msg import JointTrajectory
        src = _add_source(node, name, priority)
        src.last_received_stamp = node.get_clock().now()
        src.latest_reference = JointTrajectory()
        src.latest_stream_route = em.ROUTE_JSPC
        return src

    def _add_pose(self, node, name, priority):
        from geometry_msgs.msg import PoseStamped
        src = _add_source(node, name, priority)
        src.last_received_stamp = node.get_clock().now()
        src.latest_pose_target = PoseStamped()
        src.latest_stream_route = em.ROUTE_TSKPC
        return src

    def _wire_pubs(self, node):
        node._ref_pub = _FakePub()
        node._pose_ref_pub = _FakePub()
        node._pose_chunk_ref_pub = _FakePub()
        node._twist_ref_pub = _FakePub()

    def test_active_route_reports_winner_route(self, em_node):
        self._add_joint(em_node, "planner", priority=5)
        self._add_pose(em_node, "teleop", priority=10)
        assert em_node._active_streaming_route() == em.ROUTE_TSKPC

    def test_jspc_winner_suppresses_tskpc(self, em_node):
        self._wire_pubs(em_node)
        self._add_joint(em_node, "planner", priority=10)
        self._add_pose(em_node, "teleop", priority=5)
        em_node._state = em.STATE_ACTIVE

        em_node._publish_from_arbitrator()
        em_node._publish_pose_output()

        assert len(em_node._ref_pub.published) == 1       # JSPC emitted
        assert len(em_node._pose_ref_pub.published) == 0  # TSKPC suppressed

    def test_tskpc_winner_suppresses_jspc(self, em_node):
        self._wire_pubs(em_node)
        self._add_joint(em_node, "planner", priority=5)
        self._add_pose(em_node, "teleop", priority=10)
        em_node._state = em.STATE_ACTIVE

        em_node._publish_from_arbitrator()
        em_node._publish_pose_output()

        assert len(em_node._ref_pub.published) == 0       # JSPC suppressed
        assert len(em_node._pose_ref_pub.published) == 1  # TSKPC emitted


class TestStatusSnapshot:
    def test_aggregates_and_winning_source(self):
        now = rclpy.clock.Clock().now()
        src = _fresh_source("teleop", priority=10, now=now)
        src.decode_fail_count = 2
        src.normalize_drop_count = 3
        src.stale_count = 4
        src.invalid_stamp_count = 5
        src.last_normalize_latency_ms = 1.23456

        snapshot = status_mod.build_status_snapshot(
            sources={"teleop": src},
            now=now,
            state=em.STATE_ACTIVE,
            winning_source="teleop",
            stale_timeout_s=0.5,
            joint_names=["joint1", "joint2"],
            consecutive_fresh_count=7,
        )

        assert snapshot["schema_version"] == 1
        assert snapshot["active_source"] == "teleop"
        assert snapshot["winning_source_priority"] == 10
        assert snapshot["decode_fail_count"] == 2
        assert snapshot["normalize_drop_count"] == 3
        assert snapshot["stale_count"] == 4
        assert snapshot["invalid_stamp_count"] == 5
        assert snapshot["normalize_latency_ms"] == 1.235
        assert snapshot["consecutive_fresh_count"] == 7
        assert snapshot["sources"]["teleop"]["active"] is True
        assert snapshot["sources"]["teleop"]["fresh"] is True

    def test_streaming_route_fields_exposed(self):
        now = rclpy.clock.Clock().now()
        src = _fresh_source("teleop", priority=10, now=now)
        snapshot = status_mod.build_status_snapshot(
            sources={"teleop": src},
            now=now,
            state=em.STATE_ACTIVE,
            winning_source=None,
            stale_timeout_s=0.5,
            joint_names=["joint1"],
            consecutive_fresh_count=0,
            streaming_winner="teleop",
            active_streaming_route=em.ROUTE_TSKPC,
        )
        assert snapshot["streaming_winner"] == "teleop"
        assert snapshot["active_streaming_route"] == em.ROUTE_TSKPC

    def test_streaming_route_fields_default_none(self):
        now = rclpy.clock.Clock().now()
        snapshot = status_mod.build_status_snapshot(
            sources={},
            now=now,
            state=em.STATE_IDLE,
            winning_source=None,
            stale_timeout_s=0.5,
            joint_names=["joint1"],
            consecutive_fresh_count=0,
        )
        assert snapshot["streaming_winner"] is None
        assert snapshot["active_streaming_route"] is None

    def test_consecutive_valid_count_from_winning_source(self):
        now = rclpy.clock.Clock().now()
        src = _fresh_source("teleop", priority=10, now=now)
        src.consecutive_valid_count = 9
        snapshot = status_mod.build_status_snapshot(
            sources={"teleop": src},
            now=now,
            state=em.STATE_ACTIVE,
            winning_source="teleop",
            stale_timeout_s=0.5,
            joint_names=["joint1"],
            consecutive_fresh_count=0,
        )
        # Was previously hard-coded to 0; now reflects the winning source.
        assert snapshot["consecutive_valid_count"] == 9
        assert snapshot["sources"]["teleop"]["consecutive_valid_count"] == 9

    def test_stale_source_marked_inactive_and_not_fresh(self):
        now = rclpy.clock.Clock().now()
        src = _fresh_source("planner", priority=0, now=now, seconds_ago=1.0)
        src.inactive_timeout_s = 0.2

        snapshot = status_mod.build_status_snapshot(
            sources={"planner": src},
            now=now,
            state=em.STATE_DEGRADED,
            winning_source=None,
            stale_timeout_s=0.5,
            joint_names=["joint1"],
            consecutive_fresh_count=0,
        )

        assert snapshot["active_source"] == "planner"
        assert snapshot["winning_source"] is None
        assert snapshot["winning_source_priority"] is None
        assert snapshot["sources"]["planner"]["active"] is False
        assert snapshot["sources"]["planner"]["fresh"] is False


class TestStampValidation:
    def test_zero_stamp_allowed(self):
        from builtin_interfaces.msg import Time
        now = rclpy.clock.Clock().now()
        result = validation.validate_reference_stamp(
            stamp=Time(),
            now=now,
            reject_zero_stamp=False,
            max_future_s=0.1,
            stale_timeout_s=0.5,
        )
        assert result.valid is True
        assert result.latency_ms is None

    def test_zero_stamp_rejected(self):
        from builtin_interfaces.msg import Time
        now = rclpy.clock.Clock().now()
        result = validation.validate_reference_stamp(
            stamp=Time(),
            now=now,
            reject_zero_stamp=True,
            max_future_s=0.1,
            stale_timeout_s=0.5,
        )
        assert result.valid is False
        assert result.code == validation.STAMP_ZERO_REJECTED

    def test_future_stamp_rejected(self):
        now = rclpy.clock.Clock().now()
        future = (now + rclpy.duration.Duration(seconds=1.0)).to_msg()
        result = validation.validate_reference_stamp(
            stamp=future,
            now=now,
            reject_zero_stamp=False,
            max_future_s=0.1,
            stale_timeout_s=0.5,
        )
        assert result.valid is False
        assert result.code == validation.STAMP_FUTURE_REJECTED

    def test_stale_stamp_rejected(self):
        now = rclpy.clock.Clock().now()
        stale = (now - rclpy.duration.Duration(seconds=1.0)).to_msg()
        result = validation.validate_reference_stamp(
            stamp=stale,
            now=now,
            reject_zero_stamp=False,
            max_future_s=0.1,
            stale_timeout_s=0.5,
        )
        assert result.valid is False
        assert result.code == validation.STAMP_STALE_REJECTED

    def test_valid_stamp_reports_latency(self):
        now = rclpy.clock.Clock().now()
        stamp = (now - rclpy.duration.Duration(seconds=0.1)).to_msg()
        result = validation.validate_reference_stamp(
            stamp=stamp,
            now=now,
            reject_zero_stamp=False,
            max_future_s=0.1,
            stale_timeout_s=0.5,
        )
        assert result.valid is True
        assert result.latency_ms == pytest.approx(100.0, abs=1.0)


class TestIngestHelpers:
    def test_decode_joint_reference_dispatches_target(self):
        msg = _make_joint_state(["joint2", "joint1"], [0.2, 0.1])
        out = ingest.decode_joint_reference(
            contract_type=em.CONTRACT_JOINT_TARGET,
            msg=msg,
            joint_names=_joint_names(2),
            joint_limits_lower=[],
            joint_limits_upper=[],
            clock=rclpy.clock.Clock(),
            reject_zero_stamp=False,
        )
        assert out is not None
        assert out.joint_names == _joint_names(2)
        assert list(out.points[0].positions) == [0.1, 0.2]

    def test_joint_reference_drop_reason_detects_empty_output(self):
        from trajectory_msgs.msg import JointTrajectory
        assert ingest.joint_reference_drop_reason(None) == "decode failed"
        assert (
            ingest.joint_reference_drop_reason(JointTrajectory())
            == "normalize drop: empty joint_names or points"
        )

    def test_apply_valid_joint_reference_updates_source_state(self):
        now = rclpy.clock.Clock().now()
        src = em.SourceState(name="teleop", priority=10, inactive_timeout_s=0.3)
        normalized = _make_joint_trajectory(
            _joint_names(2), [(0.0, [0.1, 0.2], None)]
        )

        ingest.apply_valid_joint_reference(
            source=src,
            normalized=normalized,
            contract_type=em.CONTRACT_JOINT_CHUNK,
            now=now,
            latency_ms=1.25,
        )

        assert src.latest_reference is normalized
        assert src.latest_contract_type == em.CONTRACT_JOINT_CHUNK
        assert src.last_received_stamp == now
        assert src.consecutive_valid_count == 1
        assert src.last_normalize_latency_ms == 1.25

    def test_decode_pose_reference_initializes_delta_reference(self):
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.pose.position.x = 0.5
        msg.pose.orientation.w = 1.0

        decoded, is_chunk, ref_pose_update = ingest.decode_pose_reference(
            contract_type=em.CONTRACT_POSE_TARGET,
            msg=msg,
            source_ref_pose=None,
        )

        assert decoded is msg
        assert is_chunk is False
        assert ref_pose_update is not None
        assert ref_pose_update[0] == pytest.approx(0.5)

    def test_apply_valid_pose_reference_prefers_latest_shape(self):
        from geometry_msgs.msg import Pose, PoseArray, PoseStamped
        now = rclpy.clock.Clock().now()
        src = em.SourceState(name="teleop", priority=10, inactive_timeout_s=0.3)

        target = PoseStamped()
        target.pose.orientation.w = 1.0
        ingest.apply_valid_pose_reference(
            source=src,
            decoded=target,
            is_chunk=False,
            now=now,
            ref_pose_update=None,
            latency_ms=2.5,
        )
        assert src.latest_pose_target is target
        assert src.latest_pose_chunk is None
        assert src.last_normalize_latency_ms == 2.5

        chunk = PoseArray()
        p = Pose()
        p.orientation.w = 1.0
        chunk.poses = [p]
        ingest.apply_valid_pose_reference(
            source=src,
            decoded=chunk,
            is_chunk=True,
            now=now,
            ref_pose_update=None,
            latency_ms=1.5,
        )
        assert src.latest_pose_target is None
        assert src.latest_pose_chunk is chunk
        assert src.consecutive_valid_count == 2
        assert src.last_normalize_latency_ms == 1.5

    def test_decode_twist_reference_validates_finite_values(self):
        from geometry_msgs.msg import TwistStamped

        msg = TwistStamped()
        msg.twist.linear.x = 0.1
        msg.twist.angular.z = 0.2

        decoded = ingest.decode_twist_reference(
            contract_type=em.CONTRACT_TWIST_TARGET,
            msg=msg,
        )
        assert decoded is msg

        msg.twist.linear.y = math.nan
        assert ingest.decode_twist_reference(
            contract_type=em.CONTRACT_TWIST_TARGET,
            msg=msg,
        ) is None

    def test_apply_valid_twist_reference_updates_source_state(self):
        from geometry_msgs.msg import TwistStamped

        now = rclpy.clock.Clock().now()
        src = em.SourceState(name="teleop", priority=10, inactive_timeout_s=0.3)
        msg = TwistStamped()

        ingest.apply_valid_twist_reference(
            source=src,
            decoded=msg,
            now=now,
            latency_ms=0.75,
        )

        assert src.latest_twist_target is msg
        assert src.last_received_stamp == now
        assert src.consecutive_valid_count == 1
        assert src.last_normalize_latency_ms == 0.75


# ---------------------------------------------------------------------------
# _select_winning_source
# ---------------------------------------------------------------------------

class TestWinningSource:
    def test_no_sources_returns_none(self, em_node):
        assert em_node._select_winning_source() is None

    def test_single_fresh_wins(self, em_node):
        s = _add_source(em_node, "teleop", priority=10)
        _stamp_source(s, em_node)
        assert em_node._select_winning_source() == "teleop"

    def test_higher_priority_wins(self, em_node):
        lo = _add_source(em_node, "planner", priority=5)
        hi = _add_source(em_node, "teleop", priority=10)
        _stamp_source(lo, em_node)
        _stamp_source(hi, em_node)
        assert em_node._select_winning_source() == "teleop"

    def test_stale_excluded(self, em_node):
        s = _add_source(em_node, "teleop", priority=10)
        _stamp_source(s, em_node, seconds_ago=999.0)  # way beyond stale_timeout
        assert em_node._select_winning_source() is None

    def test_inactive_source_excluded(self, em_node):
        s = _add_source(em_node, "teleop", priority=10, inactive_timeout_s=0.1)
        _stamp_source(s, em_node, seconds_ago=0.2)  # > inactive_timeout but < stale
        assert em_node._select_winning_source() is None

    def test_no_reference_excluded(self, em_node):
        s = _add_source(em_node, "teleop", priority=10)
        s.last_received_stamp = em_node.get_clock().now()
        # latest_reference is left as None
        assert em_node._select_winning_source() is None

    def test_tied_priority_takes_most_recent(self, em_node):
        a = _add_source(em_node, "a", priority=5)
        b = _add_source(em_node, "b", priority=5)
        _stamp_source(a, em_node, seconds_ago=0.1)
        _stamp_source(b, em_node, seconds_ago=0.01)  # b is fresher
        assert em_node._select_winning_source() == "b"


# ---------------------------------------------------------------------------
# _should_switch_to
# ---------------------------------------------------------------------------

class TestShouldSwitchTo:
    def test_none_returns_false(self, em_node):
        assert em_node._should_switch_to(None) is False

    def test_no_current_winner_returns_true(self, em_node):
        _add_source(em_node, "teleop", priority=10)
        assert em_node._should_switch_to("teleop") is True

    def test_same_source_returns_true(self, em_node):
        _add_source(em_node, "teleop", priority=10)
        em_node._winning_source = "teleop"
        assert em_node._should_switch_to("teleop") is True

    def test_higher_priority_preempts_during_hold(self, em_node):
        _add_source(em_node, "planner", priority=5)
        _add_source(em_node, "teleop", priority=10)
        em_node._winning_source = "planner"
        # Set hold window far in the future
        em_node._hold_until = em_node.get_clock().now() + rclpy.duration.Duration(
            seconds=60.0)
        # Higher priority can preempt even during hold
        assert em_node._should_switch_to("teleop") is True

    def test_same_priority_blocked_during_hold(self, em_node):
        _add_source(em_node, "teleop", priority=5)
        _add_source(em_node, "planner", priority=5)
        em_node._winning_source = "teleop"
        em_node._hold_until = em_node.get_clock().now() + rclpy.duration.Duration(
            seconds=60.0)
        assert em_node._should_switch_to("planner") is False

    def test_any_switch_allowed_after_hold_expires(self, em_node):
        _add_source(em_node, "teleop", priority=5)
        _add_source(em_node, "planner", priority=5)
        em_node._winning_source = "teleop"
        # Hold already expired (set to past)
        em_node._hold_until = em_node.get_clock().now() - rclpy.duration.Duration(
            seconds=1.0)
        assert em_node._should_switch_to("planner") is True

    def test_no_hold_window_free_switch(self, em_node):
        _add_source(em_node, "teleop", priority=5)
        _add_source(em_node, "planner", priority=5)
        em_node._winning_source = "teleop"
        em_node._hold_until = None
        assert em_node._should_switch_to("planner") is True


# ---------------------------------------------------------------------------
# SourceState defaults
# ---------------------------------------------------------------------------

class TestSourceState:
    def test_defaults(self):
        src = em.SourceState(name="test", priority=0, inactive_timeout_s=0.3)
        assert src.name == "test"
        assert src.priority == 0
        assert src.inactive_timeout_s == 0.3
        assert src.last_received_stamp is None
        assert src.latest_reference is None
        assert src.decode_fail_count == 0
        assert src.stale_count == 0
        assert src.last_error == ""
