import math

import pytest

from mpd_dynamic_planner_adapter.world_demo_node import (
    _TO_DRAWER_CROSSING_SPECS,
    _scenario_objects,
)


_LOCAL_PATH_TANGENTS_XY = (
    (-0.78465003, 0.61993897),
    (0.05323737, 0.99858189),
)


def test_to_drawer_obstacles_cross_successively_and_perpendicularly():
    assert len(_TO_DRAWER_CROSSING_SPECS) == 2
    assert _TO_DRAWER_CROSSING_SPECS[0][3] < _TO_DRAWER_CROSSING_SPECS[1][3]

    for spec, tangent in zip(_TO_DRAWER_CROSSING_SPECS, _LOCAL_PATH_TANGENTS_XY):
        object_id, anchor, direction, crossing_time, speed = spec
        objects_at_crossing = _scenario_objects(
            "to_drawer_bridge_crossing", crossing_time
        )
        obstacle = next(item for item in objects_at_crossing if item["id"] == object_id)

        assert obstacle["position"] == pytest.approx(anchor)
        assert math.hypot(*direction) == pytest.approx(1.0, abs=1.0e-7)
        assert direction[0] * tangent[0] + direction[1] * tangent[1] == pytest.approx(
            0.0, abs=1.0e-7
        )

        before = next(
            item
            for item in _scenario_objects(
                "to_drawer_bridge_crossing", crossing_time - 0.1
            )
            if item["id"] == object_id
        )
        after = next(
            item
            for item in _scenario_objects(
                "to_drawer_bridge_crossing", crossing_time + 0.1
            )
            if item["id"] == object_id
        )
        measured_speed = math.hypot(
            after["position"][0] - before["position"][0],
            after["position"][1] - before["position"][1],
        ) / 0.2
        assert measured_speed == pytest.approx(speed)


def test_to_drawer_crossing_publishes_two_distinct_known_objects():
    objects = _scenario_objects("to_drawer_bridge_crossing", 0.0)

    assert [item["id"] for item in objects] == [
        "demo-box-crossing-1",
        "demo-box-crossing-2",
    ]
    assert all(item["local_sdf"]["type"] == "box" for item in objects)
    assert all(item["orientation_xyzw"] == [0.0, 0.0, 0.0, 1.0] for item in objects)


def test_existing_world_demo_scenarios_still_publish_one_object():
    for scenario in ("safe_far", "crossing", "to_drawer_crossing"):
        assert len(_scenario_objects(scenario, 1.0)) == 1


def test_unknown_world_demo_scenario_is_rejected():
    with pytest.raises(ValueError, match="unknown scenario"):
        _scenario_objects("not-a-scenario", 0.0)
