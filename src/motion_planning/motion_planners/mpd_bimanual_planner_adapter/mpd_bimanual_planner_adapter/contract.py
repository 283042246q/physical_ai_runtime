from __future__ import annotations

import math

JOINT_NAMES = tuple([f"Joint{i}_L" for i in range(1, 8)] + [f"Joint{i}_R" for i in range(1, 8)])
TASK_MODES = {0, 1, 2, 3}


def validate_goal(goal):
    if goal.task_mode not in TASK_MODES:
        raise ValueError("task_mode must be LEFT_ONLY, RIGHT_ONLY, DUAL_INDEPENDENT or COOPERATIVE_RIGID")
    if list(goal.joint_names) != list(JOINT_NAMES):
        raise ValueError("joint_names must be in canonical left-then-right order")
    if len(goal.q_goal) not in (0, 14) or any(not math.isfinite(float(value)) for value in goal.q_goal):
        raise ValueError("q_goal must be empty or contain fourteen finite values")
    if goal.planning_budget_s <= 0.0 or not math.isfinite(goal.planning_budget_s):
        raise ValueError("planning_budget_s must be finite and positive")
    if goal.task_mode == 3 and not (goal.has_object_goal_pose or len(goal.q_goal) == 14):
        raise ValueError("cooperative goal needs object pose or q_goal")
    return True
