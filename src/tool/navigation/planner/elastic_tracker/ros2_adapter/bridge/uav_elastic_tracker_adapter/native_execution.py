"""Standalone Elastic polynomial -> ordinary FollowTimedPath Goal/modify.

Historical entry names are retained. No framework runtime, source-reference
publisher, or high-rate adapter sampler is used.
"""
import json
import math
import uuid
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from as2_msgs.msg import TrajectorySetpoints
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from quadrotor_interfaces.msg import PolyTraj
from .execution_client import FleetFollowPathClient
from .node import Adapter
from .output_node import message
from .core import stamp_ns, suffix, yaw_at, InsufficientCoverage
from .contracts import check_time, InputRejected
from .timing import trace
from .native_message import trajectory_record


class NativeAdapter(Adapter):
    def __init__(self):
        super().__init__()
        self.declare_parameter('execution_mode', 'timed_path')
        if self.get_parameter('execution_mode').value != 'timed_path':
            raise ValueError('standalone entry requires execution_mode=timed_path')
        self.enabled = False
        self.phase = 'inactive'
        self.client = None
        self.position = None
        self.error = ''
        self.queued = self.last_curve = self.first_start = None
        self.reference_ready = False
        self.initial_handoff = None
        self.last_source = self.handoff_context = None
        self.submission = None
        self.coverage_end_ns = self.coverage_stop = None
        self.source_count = self.submitted = self.late_updates = 0
        ns = self.config['deployment'][self.vehicle]['namespace'].strip('/')
        self.state_publisher = self.create_publisher(String, '/' + ns + '/elastic_adapter/state', 10)
        self.create_subscription(TrajectorySetpoints, '/' + ns + '/motion_reference/trajectory',
                                 self.on_reference, qos_profile_sensor_data)
        self.create_service(SetBool, '/' + ns + '/elastic_adapter/enable', self.enable)
        self.native_topic = self.declare_parameter('native_topic', '').value
        self.execution_handoff = self.declare_parameter('execution_handoff', False).value
        if self.execution_handoff:
            if not self.native_topic:
                raise ValueError('execution_handoff requires a direct PolyTraj topic')
            self.accepted = self.create_publisher(Int32, '/elastic/planning/execution_accepted', 10)
            self.discarded = self.create_publisher(Int32, '/elastic/planning/execution_discarded', 10)
        if self.native_topic:
            self.native_trigger = self.create_publisher(PoseStamped, '/triger',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(PolyTraj, self.native_topic, self.on_native_trajectory,
                                     QoSProfile(depth=1))

    def odom(self, msg):
        super().odom(msg)
        if self.odom_stamp == stamp_ns([msg.header.stamp.sec, msg.header.stamp.nanosec]):
            p = msg.pose.pose.position
            self.position = [p.x, p.y, p.z]

    def input_ready(self):
        if self.position is None or self.target_stamp is None or self.odom_stamp is None:
            return False
        now = self.get_clock().now().nanoseconds
        try:
            check_time(self.target_stamp, now, max_age_ns=round(self.parameters['target_timeout'] * 1e9))
            check_time(self.odom_stamp, now)
        except InputRejected:
            return False
        return True

    def owner_conflict(self):
        names = self.get_node_names()
        return ('planner_framework' in names or names.count(self.get_name()) > 1 or
                (self.get_name() != 'elastic_tracker_adapter' and 'elastic_tracker_adapter' in names))

    def enable(self, request, response):
        if not request.data:
            self.enabled = False
            self.queued = None
            self.phase = 'stopping'
            if self.client:
                self.client.close()
            response.success = True
            response.message = 'Stop requested; wait for state.stop_confirmed'
        elif self.enabled:
            response.success = True
        elif self.owner_conflict():
            response.success = False
            response.message = self.error = 'EXECUTION_OWNER_CONFLICT: standalone requires exclusive ownership'
        elif (self.client is not None and not self.client.quiescent) or not self.input_ready() or (
                self.pending_rpc is not None and not self.pending_rpc.done()):
            response.success = False
            response.message = 'Fresh inputs and a stopped previous task are required'
        else:
            if self.client:
                for action in self.client.actions.values():
                    action.destroy()
                for service in (*self.client.modifiers.values(), *self.client.stoppers.values()):
                    self.destroy_client(service)
                for subscription in self.client.stop_subscriptions:
                    self.destroy_subscription(subscription)
            self.task = str(uuid.uuid4())
            self.config['task']['task_id'] = self.task
            self.last_id = self.pending_rpc = None
            self.queued = self.last_curve = self.first_start = None
            self.reference_ready = False
            self.initial_handoff = None
            self.last_source = self.handoff_context = None
            self.submission = None
            self.coverage_end_ns = self.coverage_stop = None
            self.source_count = self.submitted = self.late_updates = 0
            self.error = ''
            self.failed = False
            self.phase = 'waiting_for_plan'
            self.client = FleetFollowPathClient(self, self.config, self.client_failure,
                                               on_ready=self.execution_ready)
            self.enabled = True
            if self.native_topic:
                trigger = PoseStamped()
                trigger.header.stamp = self.get_clock().now().to_msg()
                trigger.pose.orientation.w = 1.
                self.native_trigger.publish(trigger)
            response.success = True
        return response

    def client_failure(self, reason, detail):
        self.fail(reason, detail)

    def fail(self, reason, detail):
        self.phase = 'failed'
        self.error = reason + ':' + detail
        self.enabled = False
        self.queued = None
        self.get_logger().error(self.error)
        if reason == 'EXECUTION_FAILED' and self.handoff_context is not None:
            self.get_logger().error('HANDOFF_CONTEXT '+json.dumps(dict(
                self.handoff_context, failed_now_ns=self.get_clock().now().nanoseconds)))
        if self.client:
            self.client.close()
        super().fail(reason, detail)

    def on_reference(self, msg):
        if (self.enabled and self.client and self.client.revision > 0 and self.first_start is not None
                and msg.header.frame_id == self.parameters['world_frame'] and len(msg.setpoints) == 1
                and stamp_ns([msg.header.stamp.sec, msg.header.stamp.nanosec]) >= self.first_start):
            first_reference = not self.reference_ready
            self.reference_ready = True
            if first_reference and self.parameters.get('source_driven_rpc', False):
                self.execution_ready()

    def execution_ready(self):
        if (getattr(self, 'execution_handoff', False) and self.submission is not None
                and not self.client.pending and not self.client.closed and not self.client.failed):
            native, rows, valid = self.submission
            self.submission = None
            self.last_source = native
            self.last_curve = (rows, valid)
            if self.first_start is None:
                self.first_start = valid
            self.accepted.publish(Int32(data=native['traj_id']))
            trace(self, 'native_offer_accepted', source_id=native['traj_id'],
                  revision=self.client.revision, valid_from_ns=valid)
        self.submit_queued()
        if self.native_topic:
            return
        if (self.enabled and self.parameters.get('source_driven_rpc', False) and
                self.client and not self.client.closed and not self.client.failed and
                not self.client.pending and (not self.client.revision or self.reference_ready)):
            # The proxy waits for a publication, so this is producer-paced,
            # single-flight I/O rather than an immediate cached-response loop.
            super().tick()

    def on_native_trajectory(self, message):
        if not self.enabled or not self.client or self.client.closed or self.client.failed:
            return
        # Same demand boundary as fresh RPC: wait for execution ACK and the
        # first live reference, then consume the next publication. Hover still
        # preempts even an unacknowledged submission.
        if not message.hover and (self.client.pending or
                (self.client.revision and not self.reference_ready)):
            if getattr(self, 'execution_handoff', False) and message.traj_id != self.last_id:
                self.discard_offer(message.traj_id, 'waiting_for_execution')
            return
        now = self.get_clock().now().nanoseconds
        trace(self, 'dds_rx', source_id=message.traj_id,
              native_start_ns=message.start_time.sec*1_000_000_000+message.start_time.nanosec)
        if not self._fresh_inputs(now, 'current_input'):
            return
        self._publish_result({'trajectory':trajectory_record(message, now)}, now)
        if not self.failed:
            self.execution_ready()

    def discard_offer(self, source_id, reason):
        self.discarded.publish(Int32(data=source_id))
        trace(self, 'native_offer_discarded', source_id=source_id, reason=reason)

    def on_rpc_ready(self):
        if not self.enabled:
            if self.pending_rpc is not None and self.pending_rpc.done():
                self.pending_rpc = None
            return
        super().on_rpc_ready()
        if not self.failed:
            self.execution_ready()

    def _publish_result(self, result, now_ns):
        native = result.get('trajectory')
        if native is None or native['traj_id'] == self.last_id:
            return
        if self.last_id is not None and native['traj_id'] < self.last_id:
            self.fail('UPSTREAM_RESET', 'new task required')
            return
        self.source_count += 1
        self.last_id = native['traj_id']
        if native['hover']:
            if len(native['hover_p']) != 3 or not all(map(math.isfinite, native['hover_p'])):
                self.fail('INVALID_SOURCE_PLAN', 'invalid hover position')
                return
            # Preempt even an unacknowledged update; never invent a final path
            # or restart the task automatically after a native stop intent.
            self.queued = None
            self.phase = 'hovering'
            self.client.close()
        elif not self.client.closed:
            self.queued = native
            trace(self, 'curve_queued', source_id=native['traj_id'], pending=bool(self.client.pending))

    def submit_queued(self):
        if (self.queued is None or not self.client.ready or self.client.pending or self.client.closed
                or (self.client.revision and not self.reference_ready)):
            return
        native = self.queued
        now = self.get_clock().now().nanoseconds
        first = self.client.revision == 0
        lead = self.parameters['valid_from_lead']
        source_start = stamp_ns(native['start_time'])
        earliest = now + round(lead * 1e9)
        handoff = getattr(self, 'execution_handoff', False)
        if handoff and source_start < earliest:
            self.queued = None
            self.late_updates += 1
            self.discard_offer(native['traj_id'], 'insufficient_lead')
            return
        valid = source_start if handoff else max(earliest, source_start)
        if self.last_curve is not None and valid <= self.last_curve[1]:
            if handoff:
                self.queued = None
                self.discard_offer(native['traj_id'], 'non_monotonic_start')
                return
            # ACKs and new curves can arrive on the same ROS clock sample.
            # Keep this candidate for the next tick or replace it with a newer one.
            trace(self, 'handoff_wait_start', source_id=native['traj_id'],
                  valid_from_ns=valid, previous_valid_from_ns=self.last_curve[1])
            return
        if not self._fresh_inputs(now, 'before_handoff'):
            return
        self.queued = None
        trace(self, 'handoff_begin', source_id=native['traj_id'])
        yaw = yaw_at(self.last_curve[0], self.last_curve[1], valid) if self.last_curve else self.yaw
        try:
            rows = suffix(native, valid, self.parameters['sample_dt'], yaw)
            segment = message(self.task, self.vehicle, stamp_ns(native['generated_at']), valid, rows)
            trace(self, 'handoff_converted', source_id=native['traj_id'], valid_from_ns=valid)
        except InsufficientCoverage:
            self.late_updates += 1
            if handoff:
                self.discard_offer(native['traj_id'], 'insufficient_coverage')
            return
        except (ValueError, OverflowError) as exc:
            self.fail('INVALID_SOURCE_PLAN', str(exc))
            return
        if first:
            # Adapter-side measurement; server admission uses its own later pose.
            self.initial_handoff = dict(
                now_ns=now, valid_from_ns=valid, requested_lead_s=lead,
                source_start_ns=source_start, source_id=native['traj_id'],
                measured_position_stamp_ns=self.odom_stamp, measured_position=self.position,
                first_position=rows[0]['pva'][0],
                adapter_position_delta_m=math.dist(self.position, rows[0]['pva'][0]))
        # Keep only this candidate and its acknowledged predecessor in memory.
        # Emit on failure, avoiding per-curve file I/O in the submission budget.
        self.handoff_context = dict(now_ns=now, valid_from_ns=valid, source=native,
            previous_source=self.last_source,
            previous_valid_from_ns=self.last_curve[1] if self.last_curve else None,
            previous_acknowledged_revision=self.client.revision)
        if handoff:
            self.submission = (native, rows, valid)
        self.client.submit(segment)
        if not self.client.failed and not self.client.closed:
            if not handoff:
                self.last_source = native
                self.last_curve = (rows, valid)
                self.first_start = valid if self.first_start is None else self.first_start
            self.submitted += 1
            self.phase = 'timed_path'

    def check_coverage(self):
        if not self.client or self.client.closed or self.client.failed:
            return
        # Only an acknowledged submission may extend executable coverage.
        if self.last_curve is not None and not self.client.pending:
            rows, valid = self.last_curve
            self.coverage_end_ns = valid + rows[-1]['time_ns']
        if self.coverage_end_ns is None:
            return
        now = self.get_clock().now().nanoseconds
        # YOPO's lead + reserve rule; reserve one existing adapter tick here.
        reserve = self.parameters['update_period']
        margin_ns = round((self.parameters['valid_from_lead'] + reserve) * 1e9)
        if self.coverage_end_ns - now <= margin_ns:
            self.coverage_stop = dict(reason='COVERAGE_EXHAUSTED', now_ns=now,
                end_ns=self.coverage_end_ns, remaining_s=(self.coverage_end_ns-now)/1e9,
                margin_s=margin_ns/1e9)
            self.queued = None
            self.phase = 'hovering'
            self.client.close()

    def tick(self):
        if self.owner_conflict() and not self.failed:
            self.fail('EXECUTION_OWNER_CONFLICT', 'standalone requires exclusive ownership')
        if self.client:
            self.client.tick(self.get_clock().now().nanoseconds / 1e9)
        if self.enabled:
            self.check_coverage()
            if self.native_topic:
                self._fresh_inputs(self.get_clock().now().nanoseconds, 'current_input')
            elif (not self.parameters.get('source_driven_rpc', False) or
                    (not self.client.closed and not self.client.pending and
                     (not self.client.revision or self.reference_ready))):
                super().tick()
            if not self.failed:
                self.submit_queued()
        self.state_publisher.publish(String(data=json.dumps(dict(task_id=self.task,
            execution_mode='timed_path', enabled=self.enabled, error=self.error, phase=self.phase,
            source_count=self.source_count, input_ready=self.input_ready(),
            prepared=bool(self.client and self.client.ready), revision=self.client.revision if self.client else 0,
            stop_requested=bool(self.client and self.client.cancel_started),
            stop_confirmed=self.client.quiescent if self.client and self.client.cancel_started else self.client is None,
            native_source_id=self.last_id, submitted_segments=self.submitted,
            late_updates=self.late_updates, reference_ready=self.reference_ready,
            initial_handoff=self.initial_handoff, coverage_stop=self.coverage_stop))))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = NativeAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.enabled = False
        if node.client and rclpy.ok():
            node.client.close()
            deadline = time.monotonic() + 5.
            while not node.client.quiescent and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        rclpy.try_shutdown()
