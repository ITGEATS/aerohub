"""ROS 2 endpoint for the existing six-kind newline JSON protocol.

This endpoint never publishes /clock or motion references. Waiting RPC readers
run outside the ROS executor so they cannot block trajectory/clock callbacks.
"""
import base64
from array import array
import json
import os
import signal
import socketserver
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy,
                       QoSCompatibility, qos_check_compatible)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from quadrotor_interfaces.msg import PolyTraj, OccMap3d
from .timing import trace
from .native_message import trajectory_record


class NativeRPCExit(Exception):
    """Unwind the executor before destroying its ROS context."""


class PublicationState:
    """Publication ownership is independent of ROS and uses no freshness timer."""
    def __init__(self, fresh=False):
        self.changed = threading.Condition()
        self.fresh = fresh
        self.latest = None
        self.task = None
        self.waiter = None
        self.error = None

    def publish(self, record):
        with self.changed:
            if self.error is None:
                self.latest = record
                self.changed.notify_all()

    def close(self, error='NATIVE_RPC_SHUTDOWN'):
        with self.changed:
            if self.error is None:
                self.error = error
            self.latest = None
            self.changed.notify_all()

    def read(self, task, trigger):
        with self.changed:
            if self.error:
                raise RuntimeError(self.error)
            if task != self.task:
                self.latest = None
                self.task = task
                trigger()
            if self.fresh:
                token = object()
                self.waiter = token
                before = self.latest
                self.changed.notify_all()
                self.changed.wait_for(lambda: self.latest is not before or
                                      self.waiter is not token or self.error)
                if self.error:
                    raise RuntimeError(self.error)
                if self.waiter is not token:
                    raise RuntimeError('LATEST_REQUEST_SUPERSEDED')
                self.waiter = None
            return {'ok': True, 'trajectory': self.latest}


class NativeRPC(Node):
    def __init__(self, **kwargs):
        super().__init__('elastic_tracker_native_rpc', **kwargs)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', int(os.environ.get('ELASTIC_PROXY_PORT', '28500')))
        self.declare_parameter('fresh_native_output', False)
        self.state = PublicationState(self.get_parameter('fresh_native_output').value)
        self.clock_condition = threading.Condition()
        self.clock_ns = None
        self.clock_mono_ns = None
        self.inputs = set()
        self.map_received = False
        self.fatal = False
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              liveliness=LivelinessPolicy.AUTOMATIC)
        self.trajectory_qos = reliable
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Reliable offers satisfy both native reliable and sensor-data consumers.
        self.outputs = {kind: self.create_publisher(cls, '/elastic/' + kind, reliable)
                        for kind, cls in [('odom', Odometry), ('target', Odometry), ('depth', Image)]}
        self.trigger_pub = self.create_publisher(PoseStamped, '/triger', durable)
        self.native_clock = self.create_publisher(Clock, '/elastic/native_clock', reliable)
        self.trajectory_sub = self.create_subscription(PolyTraj, '/elastic/trajectory', self.trajectory, reliable)
        self.create_subscription(OccMap3d, '/elastic/gridmap_inflate', self.map_callback, durable)
        self.create_subscription(Clock, '/clock', self.clock_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        trace(self, 'endpoint_init')

    def map_callback(self, _message):
        self.map_received = True

    def clock_callback(self, message):
        if not self.get_parameter('use_sim_time').value:
            return
        value = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        with self.clock_condition:
            if self.clock_ns is not None and value < self.clock_ns:
                # Native maps, optimizer and execution all retain epoch state.
                # Fail the whole launch, rather than accepting an old map/curve
                # in a new epoch. Explicit restart reinitializes all owners.
                self.fatal = True
                self.state.close('CLOCK_REWIND_RESTART_REQUIRED')
                self.get_logger().error('CLOCK_REWIND_RESTART_REQUIRED: restart the complete Elastic launch')
            if not self.fatal:
                # Same authoritative samples, on a private transport topic. The
                # native consumers never see a backwards sample before shutdown.
                self.native_clock.publish(message)
                self.clock_ns = value
                self.clock_mono_ns = time.monotonic_ns()
            self.clock_condition.notify_all()
        if self.fatal:
            raise NativeRPCExit()

    def trajectory(self, message):
        trace(self, 'native_rx', source_id=message.traj_id,
              native_start_ns=message.start_time.sec*1_000_000_000+message.start_time.nanosec,
              last_clock_ns=self.clock_ns, last_clock_mono_ns=self.clock_mono_ns)
        if self.get_parameter('use_sim_time').value and not self.clock_ns:
            return
        record = trajectory_record(message, self.get_clock().now().nanoseconds)
        self.state.publish(record)
        trace(self, 'native_available', source_id=message.traj_id)

    def trigger(self):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.orientation.w = 1.0
        self.trigger_pub.publish(message)

    def source_connected(self):
        # Discovery alone does not establish a compatible DDS connection.
        # Actual data reception remains separate from endpoint compatibility.
        for endpoint in self.get_publishers_info_by_topic('/elastic/trajectory'):
            if endpoint.topic_type != 'quadrotor_interfaces/msg/PolyTraj':
                continue
            compatibility, _ = qos_check_compatible(endpoint.qos_profile, self.trajectory_qos)
            if compatibility == QoSCompatibility.OK:
                return True
        return False

    @staticmethod
    def header(message, value):
        message.header.stamp.sec, message.header.stamp.nanosec = map(int, value['stamp'])
        message.header.frame_id = value['frame_id']

    def request(self, payload):
        if self.state.error:
            raise RuntimeError(self.state.error)
        kind = payload['kind']
        with self.clock_condition:
            self.inputs.add(kind)
        if kind == 'health':
            return {'ok': True, 'map_received': self.map_received,
                    'source_connected': self.source_connected(),
                    'clock_ns': self.get_clock().now().nanoseconds, 'inputs': sorted(self.inputs)}
        if kind == 'clock':
            requested = int(payload['stamp'][0]) * 1_000_000_000 + int(payload['stamp'][1])
            if not self.get_parameter('use_sim_time').value:
                raise ValueError('CLOCK_REQUEST_REQUIRES_SIM_TIME')
            # Acknowledge only after receiving the real external /clock. Never
            # echo the request onto the same ROS 2 domain or fabricate a clock.
            with self.clock_condition:
                self.clock_condition.wait_for(lambda: self.state.error or
                    (self.clock_ns is not None and self.clock_ns >= requested))
                if self.state.error:
                    raise RuntimeError(self.state.error)
            return {'ok': True}
        if self.get_parameter('use_sim_time').value and not self.clock_ns:
            raise RuntimeError('CLOCK_UNINITIALIZED')
        if kind == 'latest':
            return self.state.read(payload['task_id'], self.trigger)
        if kind in ('odom', 'target'):
            value = payload['message']
            message = Odometry()
            self.header(message, value['header'])
            message.child_frame_id = value['child_frame_id']
            message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = map(float, value['p'])
            q = message.pose.pose.orientation
            q.x, q.y, q.z, q.w = map(float, value['q'])
            message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = map(float, value['v'])
            message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z = map(float, value['w'])
            message.pose.covariance = list(map(float, value['pose_covariance']))
            message.twist.covariance = list(map(float, value['twist_covariance']))
        elif kind == 'depth':
            value = payload['message']
            message = Image()
            self.header(message, value['header'])
            for key in ('height', 'width', 'encoding', 'is_bigendian', 'step'):
                setattr(message, key, value[key])
            # Humble's generated setter validates bytes element by element in
            # Python. The supported uint8 array path preserves identical bytes
            # without holding the GIL for a full depth-frame validation loop.
            message.data = array('B', base64.b64decode(value['data'], validate=True))
        else:
            raise ValueError('unknown request kind')
        self.outputs[kind].publish(message)
        return {'ok': True}

    def stop_readers(self):
        self.state.close()
        with self.clock_condition:
            self.clock_condition.notify_all()


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main(args=None):
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, request_stop)
    node = NativeRPC()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            traced = False
            try:
                payload = json.loads(self.rfile.readline())
                traced = payload.get('kind') in ('latest', 'depth')
                if traced:
                    trace(node, 'server_request', port=self.client_address[1], kind=payload['kind'])
                response = node.request(payload)
                if traced:
                    trace(node, 'server_response_ready', port=self.client_address[1],
                          source_id=(response.get('trajectory') or {}).get('traj_id'))
            except Exception as exc:
                response = {'ok': False, 'error': str(exc)}
            try:
                self.wfile.write((json.dumps(response, allow_nan=False, separators=(',', ':')) + '\n').encode())
                if traced:
                    trace(node, 'server_response_written', port=self.client_address[1])
            except OSError:
                pass  # a superseded/expired RPC client may already have left

    try:
        with Server((node.get_parameter('host').value, node.get_parameter('port').value), Handler) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                # A Python signal handler cannot run while the main thread is
                # waiting indefinitely inside rcl_wait. Use a bounded wait that
                # advances even when simulation time and all DDS inputs stop.
                while rclpy.ok() and not stop_requested:
                    executor.spin_once(timeout_sec=0.1)
            except (NativeRPCExit, ExternalShutdownException):
                pass
            finally:
                node.stop_readers()
                server.shutdown()
                thread.join()
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    if node.fatal:
        raise SystemExit(2)
