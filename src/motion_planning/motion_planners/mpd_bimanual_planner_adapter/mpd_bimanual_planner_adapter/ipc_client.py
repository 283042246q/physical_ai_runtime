from __future__ import annotations

import json
import socket
import struct


PROTOCOL_SCHEMA_VERSION = 1
MAX_MESSAGE_BYTES = 1024 * 1024


class IpcError(RuntimeError):
    pass


class BimanualIpcClient:
    def __init__(self, socket_path: str, timeout_s: float = 1.0):
        self.socket_path, self.timeout_s = socket_path, float(timeout_s)

    def request(self, payload: dict) -> dict:
        encoded = json.dumps(
            payload, allow_nan=False, separators=(",", ":"), sort_keys=True
        ).encode()
        if not encoded or len(encoded) > MAX_MESSAGE_BYTES:
            raise IpcError("request exceeds worker protocol size limit")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stream:
            stream.settimeout(self.timeout_s)
            stream.connect(self.socket_path)
            stream.sendall(struct.pack("!I", len(encoded)) + encoded)
            header = self._read_exact(stream, 4)
            length = struct.unpack("!I", header)[0]
            if length < 1 or length > MAX_MESSAGE_BYTES:
                raise IpcError("worker response exceeds protocol size limit")
            body = self._read_exact(stream, length)
        result = json.loads(body)
        if not isinstance(result, dict):
            raise IpcError("worker response must be an object")
        if result.get("schema_version") != PROTOCOL_SCHEMA_VERSION:
            raise IpcError("worker protocol schema mismatch")
        return result

    @staticmethod
    def _read_exact(stream, length):
        body = bytearray()
        while len(body) < length:
            chunk = stream.recv(length - len(body))
            if not chunk:
                raise IpcError("worker closed during framed response")
            body.extend(chunk)
        return bytes(body)
