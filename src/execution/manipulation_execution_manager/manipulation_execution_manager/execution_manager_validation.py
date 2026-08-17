#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Lightweight validation helpers for execution manager hot paths."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy


STAMP_OK = "ok"
STAMP_ZERO_REJECTED = "zero_rejected"
STAMP_FUTURE_REJECTED = "future_rejected"
STAMP_STALE_REJECTED = "stale_rejected"


@dataclass(frozen=True)
class StampValidation:
    valid: bool
    code: str = STAMP_OK
    latency_s: Optional[float] = None
    latency_ms: Optional[float] = None


def validate_reference_stamp(
    *,
    stamp,
    now: rclpy.time.Time,
    reject_zero_stamp: bool,
    max_future_s: float,
    stale_timeout_s: float,
) -> StampValidation:
    """Validate a ROS message stamp against EM freshness policy.

    Zero stamps are accepted when configured to do so.  Non-zero stamps return
    latency in seconds and milliseconds so callers do not recompute it.
    """
    if stamp.sec == 0 and stamp.nanosec == 0:
        if reject_zero_stamp:
            return StampValidation(valid=False, code=STAMP_ZERO_REJECTED)
        return StampValidation(valid=True)

    stamped_time = rclpy.time.Time.from_msg(stamp, clock_type=now.clock_type)
    latency_s = (now - stamped_time).nanoseconds / 1e9
    latency_ms = latency_s * 1000.0

    if latency_s < -max_future_s:
        return StampValidation(
            valid=False,
            code=STAMP_FUTURE_REJECTED,
            latency_s=latency_s,
            latency_ms=latency_ms,
        )
    if latency_s > stale_timeout_s:
        return StampValidation(
            valid=False,
            code=STAMP_STALE_REJECTED,
            latency_s=latency_s,
            latency_ms=latency_ms,
        )

    return StampValidation(
        valid=True,
        latency_s=latency_s,
        latency_ms=latency_ms,
    )
