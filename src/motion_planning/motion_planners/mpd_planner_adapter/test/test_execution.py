from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory

from mpd_planner_adapter.execution import JtcHandoffManager


class ImmediateFuture:
    def __init__(self, value):
        self.value = value

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.value


class Result:
    def __init__(self, status=GoalStatus.STATUS_SUCCEEDED):
        self.status = status
        self.result = type(
            "TrajectoryResult", (), {"error_code": 0, "error_string": ""}
        )()


class Handle:
    def __init__(self, accepted=True, status=GoalStatus.STATUS_SUCCEEDED):
        self.accepted = accepted
        self.status = status
        self.cancel_count = 0

    def get_result_async(self):
        return ImmediateFuture(Result(self.status))

    def cancel_goal_async(self):
        self.cancel_count += 1


class Client:
    def __init__(self):
        self.handle = Handle()

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.goal = goal
        return ImmediateFuture(self.handle)


def test_goal_lifecycle_is_correlated_with_plan_id():
    events = []
    manager = object.__new__(JtcHandoffManager)
    manager._client = Client()
    manager._on_accepted = lambda plan_id: events.append((plan_id, "accepted"))
    manager._on_terminal = lambda plan_id, state: events.append((plan_id, state))
    manager._goal_handle = None
    manager.active_plan_id = None
    manager.pending_plan_id = None
    manager.state = "IDLE"
    manager.last_terminal_state = None
    manager.last_result_error_code = None
    manager.last_result_error_string = None

    assert manager.submit(42, JointTrajectory())
    assert manager.plan_id is None
    assert manager.state == "SUCCEEDED"
    assert manager.last_terminal_state == "SUCCEEDED"
    assert events == [(42, "accepted"), (42, "SUCCEEDED")]


def test_unavailable_server_fails_closed():
    manager = object.__new__(JtcHandoffManager)
    manager._client = Client()
    manager._client.server_is_ready = lambda: False
    manager.active_plan_id = None
    manager.pending_plan_id = None
    manager.state = "IDLE"
    assert not manager.submit(1, JointTrajectory())
    assert manager.state == "UNAVAILABLE"


def test_cancel_uses_owned_goal_handle():
    manager = object.__new__(JtcHandoffManager)
    handle = Handle()
    manager._goal_handle = handle
    manager.active_plan_id = 7
    manager.pending_plan_id = None
    manager.state = "ACTIVE"
    assert manager.cancel()
    assert handle.cancel_count == 1
    assert manager.state == "CANCELING"


def test_rejected_pending_goal_preserves_old_owned_goal():
    events = []
    old_handle = Handle()
    manager = object.__new__(JtcHandoffManager)
    manager._client = Client()
    manager._client.handle = Handle(accepted=False)
    manager._on_accepted = lambda plan_id: None
    manager._on_terminal = lambda plan_id, state: events.append((plan_id, state))
    manager._goal_handle = old_handle
    manager.active_plan_id = 5
    manager.pending_plan_id = None
    manager.state = "ACTIVE"
    manager.last_terminal_state = None
    manager.last_result_error_code = None
    manager.last_result_error_string = None
    assert manager.submit(6, JointTrajectory())
    assert manager.active_plan_id == 5
    assert manager._goal_handle is old_handle
    assert manager.state == "ACTIVE"
    assert events == [(6, "REJECTED")]


def test_aborted_goal_is_reported_for_its_plan():
    events = []
    manager = object.__new__(JtcHandoffManager)
    manager._client = Client()
    manager._client.handle = Handle(status=GoalStatus.STATUS_ABORTED)
    manager._on_accepted = lambda plan_id: events.append((plan_id, "accepted"))
    manager._on_terminal = lambda plan_id, state: events.append((plan_id, state))
    manager._goal_handle = None
    manager.active_plan_id = None
    manager.pending_plan_id = None
    manager.state = "IDLE"
    manager.last_terminal_state = None
    manager.last_result_error_code = None
    manager.last_result_error_string = None
    assert manager.submit(9, JointTrajectory())
    assert manager.state == "ABORTED"
    assert manager.active_plan_id is None
    assert events == [(9, "accepted"), (9, "ABORTED")]


def test_stop_invalidates_pending_goal_response():
    manager = object.__new__(JtcHandoffManager)
    manager._goal_handle = None
    manager.active_plan_id = None
    manager.pending_plan_id = 11
    manager.state = "SENDING"
    assert not manager.cancel()
    assert manager.pending_plan_id is None
    stale_handle = Handle()
    manager._goal_response(11, ImmediateFuture(stale_handle))
    assert stale_handle.cancel_count == 1
