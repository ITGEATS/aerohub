import json
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import unittest

from simulator.projectairsim.extension_client import (
    ProjectAirSimExtensionClient, ExtensionProtocolError, ExtensionRPCError, ExtensionTransportError,
)


class ProjectAirSimExtensionClientTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.requests = []
        cls.response = None

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                request = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                cls.requests.append((self.path, request))
                response = cls.response
                if response is None:
                    response = {'jsonrpc': '2.0', 'id': request['id'], 'result': True}
                self.send_response(200)
                self.end_headers()
                self.wfile.write(json.dumps(response).encode())

            def log_message(self, *args):
                pass

        cls.server = HTTPServer(('127.0.0.1', 0), Handler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def setUp(self):
        type(self).requests.clear()
        type(self).response = None
        self.client = ProjectAirSimExtensionClient(port=self.server.server_port, timeout=1)

    def test_wire_contract_and_independent_operations(self):
        self.client.set_title('任务甲')
        self.client.set_title('任务乙', side='right')
        self.client.set_title('', side='right')
        self.client.set_run_state('Drone2', 'running')
        self.client.control_timer('Drone2', 'start')
        self.client.show_message('准备', duration_seconds=0)
        self.client.clear_message()
        self.client.set_view('Drone2', 'FrontCamera', 'DepthVis')
        self.client.set_global_view()
        self.client.get_state()
        requests = [body for path, body in self.requests]
        self.assertTrue(all(path == '/aerohub/rpc' for path, _ in self.requests))
        self.assertEqual([r['id'] for r in requests], list(range(1, 11)))
        self.assertEqual(requests[1]['params'], {'text': '任务乙', 'side': 'right'})
        self.assertEqual(requests[2]['params'], {'text': '', 'side': 'right'})
        self.assertEqual(requests[3]['method'], 'SetRunState')
        self.assertEqual(requests[4]['params'], {'vehicle_id': 'Drone2', 'event': 'start'})
        self.assertEqual(requests[8]['params'], {'vehicle_id': '__global__'})

    def test_lifecycle_each_uses_one_rpc(self):
        self.client.start_run('Drone1')
        self.client.finish_run('Drone1')
        self.client.reset_run('Drone2')
        self.assertEqual([(r['method'], r['params']) for _, r in self.requests], [
            ('ControlRun', {'vehicle_id': 'Drone1', 'event': 'start'}),
            ('ControlRun', {'vehicle_id': 'Drone1', 'event': 'finish'}),
            ('ControlRun', {'vehicle_id': 'Drone2', 'event': 'reset'}),
        ])

    def test_server_error_preserves_code_without_retry(self):
        type(self).response = {'jsonrpc': '2.0', 'id': 1,
                               'error': {'code': -32602, 'message': 'invalid transition', 'data': 'idle'}}
        with self.assertRaises(ExtensionRPCError) as caught:
            self.client.control_timer('Drone1', 'finish')
        self.assertEqual(caught.exception.code, -32602)
        self.assertEqual(caught.exception.data, 'idle')
        self.assertEqual(len(self.requests), 1)

    def test_mismatched_response_id_rejected(self):
        type(self).response = {'jsonrpc': '2.0', 'id': 999, 'result': True}
        with self.assertRaises(ExtensionProtocolError):
            self.client.get_state()

    def test_invalid_arguments_do_not_send(self):
        for call in (lambda: self.client.set_title('a', side='middle'),
                     lambda: self.client.show_message('a', duration_seconds=float('nan')),
                     lambda: self.client.set_run_state('Drone1', 'ready')):
            with self.assertRaises(ValueError):
                call()
        self.assertEqual(self.requests, [])

    def test_transport_failure_is_explicit(self):
        def fail(*args, **kwargs):
            raise TimeoutError('timed out')
        self.client._open = fail
        with self.assertRaises(ExtensionTransportError):
            self.client.get_state()
