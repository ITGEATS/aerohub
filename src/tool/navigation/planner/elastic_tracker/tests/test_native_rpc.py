"""Real ROS 2 imports; publication condition regressions use deterministic events."""
from concurrent.futures import ThreadPoolExecutor
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import time
import base64
from unittest.mock import Mock
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from quadrotor_interfaces.msg import PolyTraj
from uav_elastic_tracker_adapter.native_rpc import PublicationState, NativeRPC, NativeRPCExit


class FreshPublicationTest(unittest.TestCase):
    def setUp(self):
        self.state = PublicationState()
        self.state.task = 'task'
        self.state.latest = {'traj_id': 1}
        self.entered = [threading.Event(), threading.Event()]
        self.calls = 0
        original = self.state.changed.wait_for

        def wait(predicate):
            self.entered[self.calls].set()
            self.calls += 1
            return original(predicate)
        self.state.changed.wait_for = wait

    def request(self):
        return self.state.read('task', lambda: self.fail('unexpected trigger'))

    def test_default_returns_original_cache(self):
        self.assertIs(self.request()['trajectory'], self.state.latest)
        self.assertEqual(self.calls, 0)

    def test_fresh_waits_for_real_next_publication(self):
        self.state.fresh = True
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(self.request)
            try:
                self.assertTrue(self.entered[0].wait(6.))
                self.assertFalse(future.done())
            finally:
                self.state.publish({'traj_id': 2})
            self.assertEqual(future.result(timeout=6.)['trajectory']['traj_id'], 2)

    def test_new_reader_supersedes_abandoned_reader(self):
        self.state.fresh = True
        with ThreadPoolExecutor(max_workers=2) as pool:
            old = pool.submit(self.request)
            self.assertTrue(self.entered[0].wait(6.))
            new = pool.submit(self.request)
            try:
                self.assertTrue(self.entered[1].wait(6.))
                with self.assertRaisesRegex(RuntimeError, 'LATEST_REQUEST_SUPERSEDED'):
                    old.result(timeout=6.)
            finally:
                self.state.publish({'traj_id': 3})
            self.assertEqual(new.result(timeout=6.)['trajectory']['traj_id'], 3)

    def test_shutdown_and_rewind_release_reader_and_clear_cache(self):
        self.state.fresh = True
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(self.request)
            self.assertTrue(self.entered[0].wait(6.))
            self.state.close('CLOCK_REWIND_RESTART_REQUIRED')
            with self.assertRaisesRegex(RuntimeError, 'CLOCK_REWIND_RESTART_REQUIRED'):
                future.result(timeout=6.)
        self.assertIsNone(self.state.latest)
        self.state.publish({'traj_id': 99})
        self.assertIsNone(self.state.latest)

    def test_new_task_clears_cache_and_triggers_once(self):
        triggers = []
        self.assertIsNone(self.state.read('new', lambda: triggers.append(1))['trajectory'])
        self.state.read('new', lambda: triggers.append(1))
        self.assertEqual(triggers, [1])


class EndpointContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.node = NativeRPC()

    def tearDown(self):
        self.node.destroy_node()

    def test_system_clock_mode_ignores_external_clock_and_rewind(self):
        self.assertFalse(self.node.get_parameter('use_sim_time').value)
        for seconds in (100, 99):
            message = Clock()
            message.clock.sec = seconds
            self.node.clock_callback(message)
        self.assertIsNone(self.node.clock_ns)
        self.assertFalse(self.node.fatal)
        self.assertIsNone(self.node.state.error)

    def test_depth_keeps_padding_endianness_and_original_bytes(self):
        # Real ROS Image conversion, padded big-endian uint16 rows including
        # zero/max values; transport must preserve rather than reinterpret data.
        payload = bytes.fromhex('00000100aabbffff1234ccdd')
        publisher = Mock()
        original = self.node.outputs['depth']
        self.node.outputs['depth'] = publisher
        try:
            result = self.node.request({'kind':'depth','message':dict(
                header={'stamp':[100,123],'frame_id':'camera_optical'},
                width=2,height=2,encoding='16UC1',is_bigendian=1,step=6,
                data=base64.b64encode(payload).decode())})
        finally:
            self.node.outputs['depth'] = original
        self.assertTrue(result['ok'])
        message = publisher.publish.call_args.args[0]
        self.assertEqual(bytes(message.data), payload)
        self.assertEqual((message.width,message.height,message.step,message.encoding,message.is_bigendian),
                         (2,2,6,'16UC1',1))
        self.assertEqual((message.header.stamp.sec,message.header.stamp.nanosec,message.header.frame_id),
                         (100,123,'camera_optical'))

    def test_simulation_rewind_is_not_forwarded_and_releases_readers(self):
        self.node.destroy_node()
        self.node = NativeRPC(parameter_overrides=[Parameter('use_sim_time', value=True)])
        message = Clock()
        message.clock.sec = 100
        self.node.clock_callback(message)
        message.clock.sec = 99
        with self.assertRaises(NativeRPCExit):
            self.node.clock_callback(message)
        self.node.stop_readers()
        self.assertEqual(self.node.clock_ns, 100_000_000_000)
        self.assertEqual(self.node.state.error, 'CLOCK_REWIND_RESTART_REQUIRED')

    def check_publisher(self, message_type, reliability, expected):
        publisher = self.node.create_publisher(message_type, '/elastic/trajectory',
                                             QoSProfile(depth=1, reliability=reliability))
        # Existing isolated fixture discovery budget; no product timeout change.
        deadline = time.monotonic() + 6.
        while not self.node.get_publishers_info_by_topic('/elastic/trajectory'):
            self.assertLess(time.monotonic(), deadline)
            rclpy.spin_once(self.node, timeout_sec=.02)
        self.assertEqual(self.node.request({'kind': 'health'})['source_connected'], expected)
        self.node.destroy_publisher(publisher)

    def test_reliable_polytraj_source_is_compatible(self):
        self.check_publisher(PolyTraj, ReliabilityPolicy.RELIABLE, True)

    def test_best_effort_source_does_not_report_connected(self):
        self.check_publisher(PolyTraj, ReliabilityPolicy.BEST_EFFORT, False)

    def test_wrong_type_metadata_does_not_report_connected(self):
        # This DDS implementation rejects conflicting types on one participant
        # at construction; test the graph metadata guard directly instead.
        with patch.object(self.node, 'get_publishers_info_by_topic', return_value=[
                SimpleNamespace(topic_type='std_msgs/msg/String')]):
            self.assertFalse(self.node.request({'kind': 'health'})['source_connected'])


@unittest.skipUnless(os.name == 'posix', 'requires the POSIX launch signal contract')
class NativeRPCProcessTest(unittest.TestCase):
    def check_exit(self, signals, pending=None):
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            address = reservation.getsockname()
        command = [sys.executable, '-c',
                   'from uav_elastic_tracker_adapter.native_rpc import main; main()',
                   '--ros-args', '-p', f'port:={address[1]}', '-p', 'fresh_native_output:=true']
        if pending == 'clock':
            command += ['-p', 'use_sim_time:=true']
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as child:
            try:
                def health():
                    with socket.create_connection(address, timeout=.2) as client, client.makefile('rb') as reply:
                        client.sendall(b'{"kind":"health"}\n')
                        return json.loads(reply.readline())

                # The existing launch discovery budget includes Windows bind-mount imports.
                deadline = time.monotonic() + 25.
                while True:
                    self.assertIsNone(child.poll(), 'RPC node exited before it was ready')
                    try:
                        self.assertTrue(health()['ok'])
                        break
                    except OSError:
                        self.assertLess(time.monotonic(), deadline, 'RPC node never became ready')
                        time.sleep(.02)

                def stop():
                    for i, signum in enumerate(signals):
                        if i:
                            time.sleep(.01)
                        child.send_signal(signum)
                    try:
                        log, _ = child.communicate(timeout=5.)
                    except subprocess.TimeoutExpired:
                        self.fail('RPC node did not exit before launch escalation, without DDS traffic')
                    self.assertEqual(child.returncode, 0, log)

                if pending is None:
                    # Allow startup discovery to settle; no test DDS traffic is sent.
                    time.sleep(.2)
                    stop()
                    return

                payload = {'kind': pending, 'task_id': 'shutdown-test', 'stamp': [1, 0]}
                with socket.create_connection(address, timeout=1.) as client:
                    client.sendall(json.dumps(payload).encode() + b'\n')
                    deadline = time.monotonic() + 6.
                    while pending not in health()['inputs']:
                        self.assertLess(time.monotonic(), deadline, 'RPC handler did not receive the request')
                        time.sleep(.02)
                    client.settimeout(.1)
                    with self.assertRaises(socket.timeout):
                        client.recv(1)  # prove the request is still waiting
                    stop()
                    client.settimeout(1.)
                    with client.makefile('rb') as reply:
                        self.assertEqual(json.loads(reply.readline()),
                                         {'ok': False, 'error': 'NATIVE_RPC_SHUTDOWN'})
            finally:
                if child.poll() is None:
                    child.kill()
                    child.communicate(timeout=5.)

    def test_sigint_exits_idle_node(self):
        self.check_exit([signal.SIGINT])

    def test_repeated_signals_release_pending_latest(self):
        self.check_exit([signal.SIGINT, signal.SIGINT, signal.SIGTERM], pending='latest')

    def test_sigterm_releases_pending_clock(self):
        self.check_exit([signal.SIGTERM], pending='clock')


if __name__ == '__main__':
    unittest.main(verbosity=2)
