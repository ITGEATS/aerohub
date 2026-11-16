"""Serial snapshot consumer. No motion publishers or navigation clients."""
import math
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from builtin_interfaces.msg import Time
from uav_object_detection_interfaces.srv import DetectSnapshot
from uav_target_tracking_interfaces.action import TrackObject
from uav_target_tracking_interfaces.msg import TargetState
from .core import Tracker, load_ekf


def stamp_value(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_msg(value):
    ns = max(0, int(round(value * 1e9)))
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


class TrackingNode(Node):
    def __init__(self):
        super().__init__('target_tracking')
        self.declare_parameter('snapshot_timeout', 1.0)
        self.declare_parameter('max_capture_age', 0.5)
        for name in ('snapshot_timeout', 'max_capture_age'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        self._ekf_class = load_ekf()  # fail startup before accepting goals if provenance is unavailable
        self._stopping = threading.Event()
        self._slot = threading.Lock()
        self._pending = None
        self._generation = 0
        self._clock_epoch = 0
        self._jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=self._clock_jump)
        group = ReentrantCallbackGroup()
        self._client = self.create_client(DetectSnapshot, 'tools/object_detection/observe', callback_group=group)
        self._server = ActionServer(self, TrackObject, 'tools/target_tracking/track',
                                    execute_callback=self.execute, goal_callback=self.goal,
                                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                                    callback_group=group)

    def _clock_jump(self, jump):
        self._clock_epoch += 1
        self.get_logger().warning(
            f'ROS clock generation {self._clock_epoch}: change={jump.clock_change}, delta_ns={jump.delta.nanoseconds}')

    def goal(self, request):
        if not request.query.strip() or not math.isfinite(request.timeout) or request.timeout <= 0:
            return GoalResponse.REJECT
        if not self._slot.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, handle):
        self._generation += 1
        generation = self._generation
        epoch = self._clock_epoch
        session = str(uuid.uuid4())
        tracker = Tracker(self._ekf_class)
        state = TargetState(session_id=session, generation=generation, status='unavailable')
        capture = 0
        attempt = 0
        deadline = time.monotonic() + handle.request.timeout
        last_clock = self.get_clock().now().nanoseconds * 1e-9
        reason = 'No usable observations'
        code = 'TIMEOUT'
        try:
            while rclpy.ok() and not self._stopping.is_set():
                now = self.get_clock().now().nanoseconds * 1e-9
                if handle.is_cancel_requested:
                    code = 'CANCELED'; reason = 'Canceled by caller'; handle.canceled(); break
                if self._clock_epoch != epoch or now < last_clock:
                    code = 'CLOCK_RESET'; reason = 'ROS clock generation changed or clock moved backward'; handle.abort(); break
                last_clock = now
                if time.monotonic() >= deadline:
                    reason = 'Observation window elapsed'; handle.succeed(); break
                # Do not release transport occupancy on cancel/timeout: a late
                # response must drain before a subsequent session issues a call.
                if self._pending is not None and self._pending.done():
                    future, token, started = self._pending, self._pending_token, self._pending_started
                    self._pending = None
                    response = future.result() if future.exception() is None else None
                    if token == generation and self._pending_clock_epoch == epoch and self._clock_epoch == epoch and response is not None:
                        result = response.result
                        stamp = stamp_value(result.header.stamp)
                        age = now - stamp
                        max_age = float(self.get_parameter('max_capture_age').value)
                        if response.success and result.position_requested and 0 <= age <= max_age and time.monotonic() - started <= self._pending_budget + 0.25:
                            points = [[d.position.x, d.position.y, d.position.z] for d in result.detections if d.position_valid]
                            try:
                                if result.detections and not points:
                                    raise ValueError('detections have no usable 3D positions')
                                observed_status = tracker.observe(points, stamp, result.header.frame_id)
                                capture = self._pending_capture
                                reason = observed_status
                            except ValueError as exc:
                                reason = str(exc)
                        else:
                            reason = response.reason or 'unavailable or stale capture'
                if self._pending is None and self._client.service_is_ready():
                    budget = min(float(self.get_parameter('snapshot_timeout').value), deadline - time.monotonic())
                    if budget > 0:
                        request = DetectSnapshot.Request(query=handle.request.query, estimate_position=True, timeout=budget)
                        self._pending = self._client.call_async(request)
                        attempt += 1
                        self._pending_capture = attempt
                        self._pending_token = generation
                        self._pending_clock_epoch = epoch
                        self._pending_started = time.monotonic()
                        self._pending_budget = budget
                status, valid, x, cov = tracker.snapshot(now)
                # Status describes the latest associated observation, not a
                # single feedback tick that an asynchronous consumer can miss.
                if valid and reason == 'tracked':
                    status = 'tracked'
                state = TargetState(session_id=session, generation=generation, capture_id=capture,
                                    status=status, valid=valid, reason=reason)
                state.header.frame_id = tracker.frame or ''
                state.header.stamp = stamp_msg(now)
                if tracker.observation_stamp is not None:
                    state.observation_stamp = stamp_msg(tracker.observation_stamp)
                    state.valid_until = stamp_msg(tracker.observation_stamp + tracker.ekf.max_blind_time)
                state.position.x, state.position.y, state.position.z = map(float, x[:3])
                state.velocity.x, state.velocity.y, state.velocity.z = map(float, x[3:])
                state.covariance = cov.reshape(-1).tolist()
                if self._clock_epoch != epoch:
                    continue
                handle.publish_feedback(TrackObject.Feedback(state=state))
                time.sleep(.02)
            else:
                code = 'SHUTDOWN'
                handle.abort()
        except Exception as exc:
            code = 'ERROR'; reason = str(exc)
            if handle.is_active:
                handle.abort()
        finally:
            self._generation += 1  # invalidate outstanding calls before releasing slot
            self._slot.release()
        state.valid = False
        state.reason = reason
        return TrackObject.Result(code=code, reason=reason, state=state)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stopping.set()
        executor.shutdown(timeout_sec=3)
        node.destroy_node()
        rclpy.try_shutdown()
