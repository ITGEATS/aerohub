"""Internal state/RPC transport base for the standalone native adapter."""
import json
import math
import os
import hashlib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from uav_planner_interfaces.msg import PlannerFailure
from .core import stamp_ns
from .transport import rpc
from .contracts import accept_odometry, InputRejected, InputDiagnostics, check_time
from .audit import record
from .timing import trace


class Adapter(Node):
    def __init__(self, node_name='elastic_tracker_adapter'):
        super().__init__(node_name)
        self.declare_parameter('resolved_config', '')
        self.declare_parameter('proxy_host', '127.0.0.1')
        self.declare_parameter('proxy_port', int(os.environ.get('ELASTIC_PROXY_PORT', '28500')))
        self.config = yaml.safe_load(Path(self.get_parameter('resolved_config').value).read_text())
        self.parameters = self.config['planner']['parameters']
        self.vehicle, = self.config['vehicles']
        self.task = self.config['task']['task_id']
        self.last_id = None
        self.target_stamp = None
        self.odom_stamp = None
        self.diagnostics = InputDiagnostics(self.get_logger())
        self.rpc_worker = ThreadPoolExecutor(max_workers=1)
        self.pending_rpc = None
        self.rpc_ready = self.create_guard_condition(self.on_rpc_ready)
        self.yaw = None
        self.failed = False
        self.seen_inputs = set()
        self.create_subscription(Odometry, self.parameters['accepted_target_topic'], self.target, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.parameters['odom_topic'], self.odom, qos_profile_sensor_data)
        self.failures = self.create_publisher(PlannerFailure, 'planner/failure', 10)
        self.telemetry = self.create_publisher(String, 'elastic_tracker/observations', 10)
        self.create_timer(self.parameters['update_period'], self.tick)
        trace(self, 'adapter_init')
        record(self, 'adapter', 'initialized', task_id=self.task,
               proxy_port=self.get_parameter('proxy_port').value,
               module_path=__file__, module_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())

    def target(self, msg):
        try:
            accepted = accept_odometry(msg, self.parameters['world_frame'],
                                      self.get_clock().now().nanoseconds, self.target_stamp)
        except InputRejected as exc:
            self.diagnostics.reject('target:'+str(exc))
            return
        self.target_stamp = accepted.stamp_ns
        if 'target' not in self.seen_inputs:
            self.seen_inputs.add('target')
            record(self, 'adapter', 'first_target', task_id=self.task, stamp_ns=self.target_stamp)

    def odom(self, msg):
        try:
            accepted = accept_odometry(msg, self.parameters['world_frame'],
                                      self.get_clock().now().nanoseconds, self.odom_stamp)
        except InputRejected as exc:
            self.diagnostics.reject('odom:'+str(exc))
            return
        self.odom_stamp = accepted.stamp_ns
        if 'odom' not in self.seen_inputs:
            self.seen_inputs.add('odom')
            record(self, 'adapter', 'first_odom', task_id=self.task, stamp_ns=self.odom_stamp)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))

    def fail(self, reason, detail):
        self.failed = True
        record(self, 'adapter', 'failure', task_id=self.task, reason=reason, detail=detail)
        msg = PlannerFailure()
        msg.task_id, msg.reason, msg.detail = self.task, reason, detail
        self.failures.publish(msg)

    def tick(self):
        if self.failed or self.target_stamp is None or self.odom_stamp is None or self.yaw is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - self.target_stamp) / 1_000_000_000
        self.telemetry.publish(String(data=json.dumps({'task_id': self.task, 'target_age_s': age,
            'native_traj_id': self.last_id, 'rejected': dict(self.diagnostics.counts)})))
        if self.pending_rpc is not None:
            if self.pending_rpc.done():
                self.on_rpc_ready()
            else:
                self._fresh_inputs(now_ns, 'while_waiting_rpc')
            return
        if not self._fresh_inputs(now_ns, 'current_input'):
            return
        self.pending_rpc = self.rpc_worker.submit(rpc, self.get_parameter('proxy_host').value,
            self.get_parameter('proxy_port').value, {'kind': 'latest', 'task_id': self.task})
        # Wake the ROS executor on completion; never touch execution state on
        # the socket worker, and never create another poll from this callback.
        def finished(_future):
            trace(self, 'rpc_future_done')
            self.rpc_ready.trigger()
        self.pending_rpc.add_done_callback(finished)

    def on_rpc_ready(self):
        trace(self, 'rpc_executor_ready')
        if self.pending_rpc is None or not self.pending_rpc.done():
            return
        future, self.pending_rpc = self.pending_rpc, None
        if self.failed:
            return
        try:
            result = future.result()
        except (OSError, RuntimeError, ValueError) as exc:
            self.diagnostics.reject('rpc_error:'+type(exc).__name__, detail=str(exc),
                                    now_ns=self.get_clock().now().nanoseconds)
            return
        # latest does not submit a pose snapshot to the planner: inputs travel on
        # their own stream. Only the newest accepted inputs define input health.
        now_ns = self.get_clock().now().nanoseconds
        if not self._fresh_inputs(now_ns, 'current_input'):
            return
        if result is not None:
            trace(self, 'rpc_result', source_id=(result.get('trajectory') or {}).get('traj_id'))
            self._publish_result(result, now_ns)

    def _fresh_inputs(self, now_ns, stage):
        pairs = [('target', self.target_stamp), ('odom', self.odom_stamp)]
        for kind, stamp in pairs:
            age = self.parameters['target_timeout'] if kind == 'target' else self.parameters['input_max_age']
            try:
                check_time(stamp, now_ns, max_age_ns=round(age*1_000_000_000),
                           max_future_ns=round(self.parameters['input_max_future']*1_000_000_000))
            except InputRejected as exc:
                reason = stage+':'+kind+':'+str(exc)
                self.diagnostics.reject(reason, source='current_input', now_ns=now_ns,
                                        stamp_ns=stamp, age_ns=now_ns-stamp,
                                        max_age_ns=round(age*1_000_000_000))
                self.fail('TARGET_TIMEOUT' if kind == 'target' else 'STATE_TIMEOUT', reason)
                return False
        return True

    def _publish_result(self, result, now_ns):
        raise NotImplementedError('native execution owns plan consumption')

    def destroy_node(self):
        self.rpc_worker.shutdown(wait=True)
        return super().destroy_node()
