"""Atomic event recorder for deterministic Marvin/Isaac dynamic replay."""
from __future__ import annotations

import json
from pathlib import Path
import threading
import time


class DynamicReplayRecorder:
    def __init__(self, path: Path, *, request_id: str):
        self.path = Path(path)
        self.request_id = str(request_id)
        self._events = []
        self._sequence = 0
        self._lock = threading.Lock()

    def record(self, event_type: str, payload: dict, *, unix_ns=None):
        with self._lock:
            self._sequence += 1
            self._events.append(
                {
                    "unix_ns": time.time_ns() if unix_ns is None else int(unix_ns),
                    "sequence": self._sequence,
                    "type": str(event_type),
                    "payload": payload,
                }
            )
            self.flush()

    def flush(self):
        payload = {
            "schema": "marvin_bimanual_dynamic_replay/v1",
            "request_id": self.request_id,
            "events": list(self._events),
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, allow_nan=False, indent=2) + "\n")
        temporary.replace(self.path)
