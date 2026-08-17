from manipulation_motion_planning.contracts import TrajectoryPlanPoint, TrajectoryPlanResult
from mpd_planner_adapter.trajectory import TimedPlan


def test_future_handoff_prediction_interpolates_position_and_velocity():
    plan = TrajectoryPlanResult(
        valid=True,
        joint_names=[f"fr3_joint{i}" for i in range(1, 8)],
        points=[
            TrajectoryPlanPoint([0.0] * 7, [0.0] * 7, 0.0),
            TrajectoryPlanPoint([1.0] * 7, [2.0] * 7, 2.0),
        ],
    )
    predicted = TimedPlan(plan, start_unix_s=10.0).predict(11.0)
    assert predicted.positions == [0.5] * 7
    assert predicted.velocities == [1.0] * 7
    assert predicted.stamp_s == 11.0
