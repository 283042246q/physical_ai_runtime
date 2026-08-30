import math
import json
from copy import deepcopy

import pytest

from mpd_dynamic_planner_adapter.world_demo_node import (
    _TO_DRAWER_CROSSING_SPECS,
    _scenario_file,
    _scenario_file_objects,
    _scenario_objects,
)


_LOCAL_PATH_TANGENTS_XY = (
    (-0.78465003, 0.61993897),
    (0.05323737, 0.99858189),
)


def test_to_drawer_obstacles_cross_successively_and_perpendicularly():
    assert len(_TO_DRAWER_CROSSING_SPECS) == 3

    for spec, tangent in zip(_TO_DRAWER_CROSSING_SPECS[:2], _LOCAL_PATH_TANGENTS_XY):
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

    object_id, anchor, direction, crossing_time, speed = _TO_DRAWER_CROSSING_SPECS[2]
    obstacle = next(
        item
        for item in _scenario_objects("to_drawer_bridge_crossing", crossing_time)
        if item["id"] == object_id
    )
    assert obstacle["position"] == pytest.approx(anchor)
    assert direction == pytest.approx((0.0, 0.0, 1.0))
    before = _scenario_objects("to_drawer_bridge_crossing", crossing_time - 0.1)[2]
    after = _scenario_objects("to_drawer_bridge_crossing", crossing_time + 0.1)[2]
    assert (after["position"][2] - before["position"][2]) / 0.2 == pytest.approx(speed)


def test_to_drawer_crossing_publishes_three_distinct_known_objects():
    objects = _scenario_objects("to_drawer_bridge_crossing", 0.0)

    assert [item["id"] for item in objects] == [
        "demo-box-crossing-1",
        "demo-box-crossing-2",
        "demo-box-crossing-3",
    ]
    assert all(item["local_sdf"]["type"] == "box" for item in objects)
    assert all(item["orientation_xyzw"] == [0.0, 0.0, 0.0, 1.0] for item in objects)


def test_existing_world_demo_scenarios_still_publish_one_object():
    for scenario in ("safe_far", "crossing", "to_drawer_crossing"):
        assert len(_scenario_objects(scenario, 1.0)) == 1


def test_unknown_world_demo_scenario_is_rejected():
    with pytest.raises(ValueError, match="unknown scenario"):
        _scenario_objects("not-a-scenario", 0.0)


def test_scenario_file_normalizes_direction_and_crosses_anchor(tmp_path):
    path = tmp_path / "scenario.json"
    path.write_text(
        json.dumps(
            {
                "schema": "mpd_todrawer_dynamic_scenario",
                "schema_version": 1,
                "frame_id": "fr3_link0",
                "objects": [
                    {
                        "id": "random-box-0",
                        "local_sdf": {"type": "box", "size_xyz": [0.1, 0.12, 0.14]},
                        "anchor_position": [-0.2, 0.4, 0.5],
                        "direction": [2.0, 0.0, 0.0],
                        "crossing_time_s": 12.0,
                        "speed_m_s": 0.2,
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "position_covariance_3x3": [
                            0.0001,
                            0.0,
                            0.0,
                            0.0,
                            0.0001,
                            0.0,
                            0.0,
                            0.0,
                            0.0001,
                        ],
                        "inflation": {
                            "mode": "linear",
                            "base_m": 0.02,
                            "horizon_rate_m_s": 0.01,
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    payload = _scenario_file(path)
    before = _scenario_file_objects(payload, 11.0)[0]
    crossing = _scenario_file_objects(payload, 12.0)[0]

    assert crossing["position"] == pytest.approx([-0.2, 0.4, 0.5])
    assert before["position"] == pytest.approx([-0.4, 0.4, 0.5])
    assert crossing["base_inflation_m"] == pytest.approx(0.02)


def test_scenario_file_supports_acceleration_curves_and_speed_variation(tmp_path):
    base = {
        "id": "motion",
        "local_sdf": {"type": "box", "size_xyz": [0.1, 0.12, 0.14]},
        "anchor_position": [0.0, 0.0, 0.5],
        "direction": [1.0, 0.0, 0.0],
        "crossing_time_s": 10.0,
        "speed_m_s": 0.2,
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "position_covariance_3x3": [
            0.0001,
            0.0,
            0.0,
            0.0,
            0.0001,
            0.0,
            0.0,
            0.0,
            0.0001,
        ],
        "inflation": {
            "mode": "linear",
            "base_m": 0.02,
            "horizon_rate_m_s": 0.01,
        },
    }
    acceleration = deepcopy(base)
    acceleration.update(
        id="acceleration",
        motion={
            "type": "constant_acceleration",
            "longitudinal_acceleration_m_s2": 0.02,
        },
    )
    curve = deepcopy(base)
    curve.update(
        id="curve",
        motion={
            "type": "sinusoidal_curve",
            "lateral_direction": [0.0, 1.0, 0.0],
            "lateral_amplitude_m": 0.1,
            "lateral_angular_frequency_rad_s": math.pi / 2.0,
            "lateral_phase_rad": 0.3,
        },
    )
    varying = deepcopy(base)
    varying.update(
        id="varying",
        motion={
            "type": "smooth_speed_variation",
            "speed_variation_amplitude_m_s": 0.04,
            "speed_variation_angular_frequency_rad_s": math.pi / 2.0,
            "speed_variation_phase_rad": -0.4,
        },
    )
    path = tmp_path / "motion-scenario.json"
    path.write_text(
        json.dumps(
            {
                "schema": "mpd_todrawer_dynamic_scenario",
                "schema_version": 2,
                "frame_id": "fr3_link0",
                "objects": [acceleration, curve, varying],
            }
        ),
        encoding="utf-8",
    )

    payload = _scenario_file(path)
    crossing = _scenario_file_objects(payload, 10.0)
    before = _scenario_file_objects(payload, 9.0)
    after = _scenario_file_objects(payload, 11.0)

    assert all(item["position"] == pytest.approx([0.0, 0.0, 0.5]) for item in crossing)
    assert before[0]["position"] == pytest.approx([-0.19, 0.0, 0.5])
    assert after[0]["position"] == pytest.approx([0.21, 0.0, 0.5])
    curve_offset = 0.1 * (math.sin(math.pi / 2.0 + 0.3) - math.sin(0.3))
    assert after[1]["position"] == pytest.approx([0.2, curve_offset, 0.5])
    varying_displacement = 0.2 + 0.04 / (math.pi / 2.0) * (
        math.sin(math.pi / 2.0 - 0.4) - math.sin(-0.4)
    )
    assert after[2]["position"] == pytest.approx(
        [varying_displacement, 0.0, 0.5]
    )


def test_scenario_file_rejects_duplicate_object_ids(tmp_path):
    path = tmp_path / "scenario.json"
    item = {
        "id": "duplicate",
        "local_sdf": {"type": "sphere", "radius": 0.1},
        "anchor_position": [0.0, 0.0, 0.5],
        "direction": [1.0, 0.0, 0.0],
        "crossing_time_s": 10.0,
        "speed_m_s": 0.1,
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "position_covariance_3x3": [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001],
        "inflation": {"mode": "linear", "base_m": 0.02, "horizon_rate_m_s": 0.01},
    }
    path.write_text(
        json.dumps(
            {
                "schema": "mpd_todrawer_dynamic_scenario",
                "schema_version": 1,
                "frame_id": "fr3_link0",
                "objects": [item, item],
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="unique"):
        _scenario_file(path)
