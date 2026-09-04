from __future__ import annotations

from .ipc_client import BimanualIpcClient


class BimanualPlannerBackend:
    def __init__(self, socket_path: str):
        self.client = BimanualIpcClient(socket_path)

    def plan(self, request: dict) -> dict:
        return self.client.request(request)
