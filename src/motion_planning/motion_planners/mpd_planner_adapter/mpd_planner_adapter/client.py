"""Small dependency-free client for the resident MPD worker."""

from __future__ import annotations

import json
from pathlib import Path
import socket
import struct
from typing import Any


PROTOCOL_SCHEMA_VERSION = 1
MAX_MESSAGE_BYTES = 1024 * 1024
_HEADER = struct.Struct("!I")


class MpdClientError(RuntimeError):
    """Transport or protocol failure.  Callers must fail closed."""


def _receive_exact(stream: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.recv(size - len(chunks))
        if not chunk:
            raise MpdClientError("MPD worker closed the socket mid-message")
        chunks.extend(chunk)
    return bytes(chunks)


def _send_message(stream: socket.socket, message: dict[str, Any]) -> None:
    try:
        payload = json.dumps(
            message, allow_nan=False, separators=(",", ":"), sort_keys=True
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise MpdClientError(f"request is not valid JSON: {error}") from error
    if not payload or len(payload) > MAX_MESSAGE_BYTES:
        raise MpdClientError(f"invalid request size: {len(payload)} bytes")
    stream.sendall(_HEADER.pack(len(payload)) + payload)


def _receive_message(stream: socket.socket) -> dict[str, Any]:
    (size,) = _HEADER.unpack(_receive_exact(stream, _HEADER.size))
    if size <= 0 or size > MAX_MESSAGE_BYTES:
        raise MpdClientError(f"invalid response size: {size} bytes")
    try:
        value = json.loads(_receive_exact(stream, size))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise MpdClientError(f"worker returned invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise MpdClientError("worker response root is not an object")
    return value


class MpdWorkerClient:
    """One connection per request, with finite connect/read timeouts."""

    def __init__(self, socket_path: str | Path, timeout_s: float = 2.0) -> None:
        self.socket_path = Path(socket_path).expanduser()
        self.timeout_s = float(timeout_s)
        if self.timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")

    def request(self, message: dict[str, Any]) -> dict[str, Any]:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stream:
                stream.settimeout(self.timeout_s)
                stream.connect(str(self.socket_path))
                _send_message(stream, message)
                response = _receive_message(stream)
        except (OSError, TimeoutError) as error:
            raise MpdClientError(f"MPD worker unavailable: {error}") from error
        if response.get("schema_version") != PROTOCOL_SCHEMA_VERSION:
            raise MpdClientError("worker protocol schema mismatch")
        return response

    def health(self) -> dict[str, Any]:
        return self.request({"schema_version": PROTOCOL_SCHEMA_VERSION, "op": "health"})

    def plan(
        self,
        request: dict[str, Any],
        *,
        request_seq: int,
        world_version: int,
        deadline_unix_ns: int | None,
    ) -> dict[str, Any]:
        return self.request(
            {
                "schema_version": PROTOCOL_SCHEMA_VERSION,
                "op": "plan",
                "request_seq": request_seq,
                "world_version": world_version,
                "deadline_unix_ns": deadline_unix_ns,
                "request": request,
            }
        )
