from __future__ import annotations


class CollisionGuard:
    SAFE = "safe"
    REPLAN = "replan"
    BRAKE = "brake"

    def __init__(
        self,
        *,
        stop_on_any_arm_fault=True,
        warning_clearance_m=0.05,
        brake_clearance_m=0.02,
    ):
        self.stop_on_any_arm_fault = bool(stop_on_any_arm_fault)
        self.triggered = False
        self.warning_clearance_m = float(warning_clearance_m)
        self.brake_clearance_m = float(brake_clearance_m)
        if self.brake_clearance_m < 0 or self.warning_clearance_m <= self.brake_clearance_m:
            raise ValueError("clearance thresholds must satisfy warning > brake >= 0")

    def classify(self, *, min_clearance_m: float, arm_fault: bool = False) -> str:
        if arm_fault and self.stop_on_any_arm_fault:
            return self.BRAKE
        if min_clearance_m <= self.brake_clearance_m:
            return self.BRAKE
        if min_clearance_m <= self.warning_clearance_m:
            return self.REPLAN
        return self.SAFE

    def update(self, *, min_clearance_m: float, threshold_m: float = 0.02, arm_fault: bool = False) -> bool:
        self.triggered = bool(arm_fault or min_clearance_m < threshold_m)
        return self.triggered
