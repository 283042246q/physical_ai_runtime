from __future__ import annotations

import json
import socket
import struct


class IpcError(RuntimeError):
    pass


class BimanualIpcClient:
    def __init__(self, socket_path: str, timeout_s: float = 1.0):
        self.socket_path, self.timeout_s = socket_path, float(timeout_s)

    def request(self, payload: dict) -> dict:
        encoded = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode()
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stream:
            stream.settimeout(self.timeout_s)
            stream.connect(self.socket_path)
            stream.sendall(struct.pack("!I", len(encoded)) + encoded)
            header = stream.recv(4)
            if len(header) != 4:
                raise IpcError("worker closed before response")
            length = struct.unpack("!I", header)[0]
            body = b""
            while len(body) < length:
                chunk = stream.recv(length - len(body))
                if not chunk:
                    raise IpcError("worker closed during response")
                body += chunk
        result = json.loads(body)
        if not isinstance(result, dict):
            raise IpcError("worker response must be an object")
        return result
