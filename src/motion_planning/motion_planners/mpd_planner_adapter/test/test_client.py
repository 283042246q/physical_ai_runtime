import json
import socket
import struct
import threading

from mpd_planner_adapter.client import MpdWorkerClient


def test_health_round_trip(tmp_path):
    path = tmp_path / "worker.sock"
    ready = threading.Event()

    def serve():
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(path))
            server.listen(1)
            ready.set()
            connection, _ = server.accept()
            with connection:
                size = struct.unpack("!I", connection.recv(4))[0]
                request = json.loads(connection.recv(size))
                assert request == {"op": "health", "schema_version": 1}
                payload = json.dumps(
                    {"schema_version": 1, "status": "OK", "state": "READY"}
                ).encode()
                connection.sendall(struct.pack("!I", len(payload)) + payload)

    thread = threading.Thread(target=serve)
    thread.start()
    assert ready.wait(1.0)
    response = MpdWorkerClient(path).health()
    thread.join(1.0)
    assert response["state"] == "READY"
