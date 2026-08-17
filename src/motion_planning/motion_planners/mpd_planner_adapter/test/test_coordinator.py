import threading

from mpd_planner_adapter.coordinator import LatestOnlyPlanner


def test_pending_slot_is_replaced_and_running_result_is_superseded():
    entered = threading.Event()
    release = threading.Event()
    calls = []

    def plan(value):
        calls.append(value)
        if value == "first":
            entered.set()
            assert release.wait(1.0)
        return value.upper()

    planner = LatestOnlyPlanner(plan)
    try:
        planner.submit(1, "first")
        assert entered.wait(1.0)
        planner.submit(2, "dropped")
        planner.submit(3, "latest")
        assert planner.pending_count == 1
        release.set()
        for _ in range(1000):
            results = planner.drain()
            if len(results) == 2:
                break
            threading.Event().wait(0.001)
        assert calls == ["first", "latest"]
        assert results[0].superseded
        assert not results[1].superseded
        assert results[1].result == "LATEST"
    finally:
        planner.close()


def test_invalidate_clears_pending_work():
    blocker_entered = threading.Event()
    release = threading.Event()

    def plan(value):
        blocker_entered.set()
        release.wait(1.0)
        return value

    planner = LatestOnlyPlanner(plan)
    try:
        planner.submit(1, 1)
        assert blocker_entered.wait(1.0)
        planner.submit(2, 2)
        planner.invalidate(3)
        assert planner.pending_count == 0
        release.set()
    finally:
        planner.close()
