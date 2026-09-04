from __future__ import annotations


class CollisionGuard:
    def __init__(self, *, stop_on_any_arm_fault=True):
        self.stop_on_any_arm_fault = bool(stop_on_any_arm_fault)
        self.triggered = False

    def update(self, *, min_clearance_m: float, threshold_m: float = 0.02, arm_fault: bool = False) -> bool:
        self.triggered = bool(arm_fault or min_clearance_m < threshold_m)
        return self.triggered
