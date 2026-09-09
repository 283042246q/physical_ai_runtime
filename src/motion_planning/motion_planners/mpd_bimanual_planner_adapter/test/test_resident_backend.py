from pathlib import Path

import pytest

from mpd_bimanual_planner_adapter.backend import BimanualPlannerBackend, WorkerRejected


class _Client:
    def __init__(self, response):
        self.response = response
        self.messages = []

    def request(self, message):
        self.messages.append(message)
        return dict(self.response)


def test_backend_constructs_versioned_plan_envelope(tmp_path):
    result_path = tmp_path / "result.json"
    result_path.write_text('{"request_id":"abc","status":"success"}')
    trajectory_path = tmp_path / "trajectory.npz"
    trajectory_path.write_bytes(b"npz")
    backend = BimanualPlannerBackend("/tmp/not-used")
    backend.client = _Client(
        {
            "schema_version": 1,
            "status": "OK",
            "request_seq": 7,
            "world_version": 4,
            "result_path": str(result_path),
            "trajectory_path": str(trajectory_path),
        }
    )
    result = backend.plan(
        {"request_id": "abc", "world_version": 4},
        request_seq=7,
        deadline_unix_ns=123,
    )
    assert result["_trajectory_path"] == str(trajectory_path)
    assert backend.client.messages[0]["op"] == "plan"
    assert backend.client.messages[0]["deadline_unix_ns"] == 123


@pytest.mark.parametrize("status", ["BUSY", "STALE", "NOT_READY", "FAULT"])
def test_backend_fails_closed_on_non_ok_status(status):
    backend = BimanualPlannerBackend("/tmp/not-used")
    backend.client = _Client(
        {"schema_version": 1, "status": status, "request_seq": 1, "world_version": 0}
    )
    with pytest.raises(WorkerRejected) as caught:
        backend.plan({"request_id": "abc", "world_version": 0}, request_seq=1)
    assert caught.value.status == status
