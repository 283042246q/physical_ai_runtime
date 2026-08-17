from mpd_planner_adapter.safe_stop_node import cancel_all_request


def test_cancel_all_request_uses_action_protocol_wildcards():
    request = cancel_all_request()
    assert bytes(request.goal_info.goal_id.uuid) == bytes(16)
    assert request.goal_info.stamp.sec == 0
    assert request.goal_info.stamp.nanosec == 0
