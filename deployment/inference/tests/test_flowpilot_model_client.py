"""Concurrency and no-retry regressions for the shared ModelClient.

Locks in two review findings (2026-11-23): the client must stay stateless per
request (safe under MultiThreadedExecutor adapters with reentrant callback
groups, e.g. YOPO's inference service), and it must never re-send a POST
after a response failure — the worker may already have executed it, and a
second execution corrupts stateful models such as WorldVLN sessions.
"""
import json
import sys
import threading
import unittest
from http.client import HTTPException
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/basic_function/inference/ros2_adapter'))
from uav_inference.model_client import ModelClient, WorkerError


class TestServer(ThreadingHTTPServer):
    # Exercise concurrent calls without the fixture's default five-SYN backlog.
    request_queue_size = 32


def start_server(handler):
    server = TestServer(("127.0.0.1", 0), handler)
    server.daemon_threads = True
    server.executions = {}
    server.lock = threading.Lock()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def endpoint(server):
    return f"http://127.0.0.1:{server.server_port}"


class EchoHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        with self.server.lock:
            self.server.executions[body["request_id"]] = (
                self.server.executions.get(body["request_id"], 0) + 1)
        self.respond(body)

    def respond(self, body):
        result = json.dumps({"request_id": body["request_id"], "ok": True}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(result)))
        self.end_headers()
        self.wfile.write(result)


class TruncatingHandler(EchoHandler):
    """Executes every request, but for poison ids truncates the response body.

    The Content-Length announces the full body while only a prefix is sent and
    the connection is closed: the worker (server) has executed, the client
    cannot know the outcome. A compliant client must fail without re-sending.
    """

    def respond(self, body):
        if body["request_id"].startswith("poison"):
            full = json.dumps({"request_id": body["request_id"], "ok": True}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(full)))
            self.end_headers()
            self.wfile.write(full[:20])
            self.close_connection = True
            return
        super().respond(body)


class ModelClientTest(unittest.TestCase):
    def test_concurrent_requests_never_cross_wires(self):
        server = start_server(EchoHandler)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        client = ModelClient(endpoint(server), timeout=10)
        errors = []

        def worker(index):
            try:
                for call in range(4):
                    name = f"conc-{index}-{call}"
                    result = client.request("/infer", {"request_id": name})
                    assert result["request_id"] == name, (result, name)
            except Exception as error:  # noqa: BLE001 - collected for the assert
                errors.append(error)

        threads = [threading.Thread(target=worker, args=(index,)) for index in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=30)
        self.assertFalse(any(thread.is_alive() for thread in threads))
        self.assertEqual(errors, [])
        self.assertEqual(len(server.executions), 32)
        self.assertEqual(set(server.executions.values()), {1})

    def test_truncated_response_fails_without_resending(self):
        server = start_server(TruncatingHandler)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        client = ModelClient(endpoint(server), timeout=10)
        # Warm the connection semantics with a normal call first.
        self.assertTrue(client.request("/infer", {"request_id": "warm"})["ok"])
        with self.assertRaises((WorkerError, HTTPException, ValueError)):
            client.request("/infer", {"request_id": "poison-1"})
        self.assertEqual(server.executions["poison-1"], 1,
                         "client re-sent a POST whose response failed")
        self.assertTrue(client.request("/infer", {"request_id": "after"})["ok"])
        self.assertEqual(server.executions["warm"], 1)
        self.assertEqual(server.executions["after"], 1)
        self.assertEqual(server.executions["poison-1"], 1)


if __name__ == "__main__":
    unittest.main()
