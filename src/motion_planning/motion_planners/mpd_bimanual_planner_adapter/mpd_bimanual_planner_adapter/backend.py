from __future__ import annotations

import json
from pathlib import Path

from .ipc_client import BimanualIpcClient


class WorkerRejected(RuntimeError):
    def __init__(self, status: str, message: str = ""):
        super().__init__(message or status)
        self.status = status


class BimanualPlannerBackend:
    def __init__(self, socket_path: str, timeout_s: float = 30.0):
        self.client = BimanualIpcClient(socket_path, timeout_s=timeout_s)

    def health(self) -> dict:
        return self.client.request({"schema_version": 1, "op": "health"})

    def plan(
        self,
        request: dict,
        *,
        request_seq: int,
        deadline_unix_ns: int | None = None,
    ) -> dict:
        world_version = int(request.get("world_version", 0))
        response = self.client.request(
            {
                "schema_version": 1,
                "op": "plan",
                "request_seq": int(request_seq),
                "world_version": world_version,
                "deadline_unix_ns": deadline_unix_ns,
                "request": request,
            }
        )
        if response.get("request_seq") != request_seq:
            raise WorkerRejected("PROTOCOL_ERROR", "response request_seq mismatch")
        if response.get("world_version") != world_version:
            raise WorkerRejected("PROTOCOL_ERROR", "response world_version mismatch")
        if response.get("status") != "OK":
            error = response.get("error") or {}
            raise WorkerRejected(
                str(response.get("status")), str(error.get("message", ""))
            )
        result_path = Path(str(response["result_path"]))
        trajectory_path = Path(str(response["trajectory_path"]))
        result = json.loads(result_path.read_text(encoding="utf-8"))
        if result.get("request_id") != request.get("request_id"):
            raise WorkerRejected("PROTOCOL_ERROR", "result request_id mismatch")
        result["_result_path"] = str(result_path)
        result["_trajectory_path"] = str(trajectory_path)
        return result

    def shutdown(self) -> dict:
        return self.client.request({"schema_version": 1, "op": "shutdown"})
