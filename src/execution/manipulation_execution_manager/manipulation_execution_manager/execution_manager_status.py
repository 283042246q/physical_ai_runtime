#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Status snapshot construction for the execution manager."""

from __future__ import annotations

from typing import Any, Dict, Optional

import rclpy

from .execution_manager_core import (
    STATE_ACTIVE,
    STATE_ARMED,
    STATE_DEGRADED,
    SourceState,
)


def build_status_snapshot(
    *,
    sources: Dict[str, SourceState],
    now: rclpy.time.Time,
    state: str,
    winning_source: Optional[str],
    stale_timeout_s: float,
    joint_names: list[str],
    consecutive_fresh_count: int,
    streaming_winner: Optional[str] = None,
    active_streaming_route: Optional[str] = None,
) -> Dict[str, Any]:
    """Build the JSON-serializable EM status payload.

    The returned shape preserves the M1-compatible aggregate fields and adds
    M2 per-source details. ``streaming_winner`` / ``active_streaming_route``
    expose the global JSPC/TSKPC arbitration result so pure-TSKPC deployments
    (where the joint-centric ``state``/``winning_source`` may read IDLE) remain
    observable.
    """
    sources_snapshot: Dict[str, Any] = {}
    agg_decode_fail = 0
    agg_normalize_drop = 0
    agg_stale = 0
    agg_invalid_stamp = 0

    for name, src in sources.items():
        elapsed_s = (
            (now - src.last_received_stamp).nanoseconds / 1e9
            if src.last_received_stamp is not None
            else float("inf")
        )
        sources_snapshot[name] = {
            "priority": src.priority,
            "active": (
                src.last_received_stamp is not None
                and elapsed_s <= src.inactive_timeout_s
            ),
            "fresh": (
                src.last_received_stamp is not None
                and elapsed_s <= stale_timeout_s
            ),
            "decode_fail_count": src.decode_fail_count,
            "normalize_drop_count": src.normalize_drop_count,
            "stale_count": src.stale_count,
            "invalid_stamp_count": src.invalid_stamp_count,
            "last_normalize_latency_ms": (
                round(src.last_normalize_latency_ms, 3)
                if src.last_normalize_latency_ms is not None
                else None
            ),
            "consecutive_valid_count": src.consecutive_valid_count,
            "last_error": src.last_error,
        }
        agg_decode_fail += src.decode_fail_count
        agg_normalize_drop += src.normalize_drop_count
        agg_stale += src.stale_count
        agg_invalid_stamp += src.invalid_stamp_count

    winning_latency: Optional[float] = None
    if winning_source is not None and winning_source in sources:
        winning_latency = sources[winning_source].last_normalize_latency_ms
        if winning_latency is not None:
            winning_latency = round(winning_latency, 3)
    elif sources:
        for src in sources.values():
            if src.last_normalize_latency_ms is not None:
                winning_latency = round(src.last_normalize_latency_ms, 3)
                break

    display_source = ""
    if state in (STATE_ARMED, STATE_ACTIVE, STATE_DEGRADED):
        if winning_source is not None:
            display_source = winning_source
        elif sources:
            display_source = next(iter(sources.keys()), "")

    return {
        "schema_version": 1,
        "active_source": display_source,
        "state": state,
        "decode_fail_count": agg_decode_fail,
        "normalize_drop_count": agg_normalize_drop,
        "stale_count": agg_stale,
        "invalid_stamp_count": agg_invalid_stamp,
        "normalize_latency_ms": winning_latency,
        "winning_source": winning_source,
        "streaming_winner": streaming_winner,
        "active_streaming_route": active_streaming_route,
        "winning_source_priority": (
            sources[winning_source].priority
            if winning_source is not None and winning_source in sources
            else None
        ),
        "sources": sources_snapshot,
        "last_error": (
            sources[winning_source].last_error
            if winning_source is not None and winning_source in sources
            else ""
        ),
        "joint_names": joint_names,
        "consecutive_valid_count": (
            sources[winning_source].consecutive_valid_count
            if winning_source is not None and winning_source in sources
            else 0
        ),
        "consecutive_fresh_count": consecutive_fresh_count,
    }
