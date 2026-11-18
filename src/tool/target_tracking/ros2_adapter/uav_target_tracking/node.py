"""Serial target-state Tool, optionally requesting turn-only Navigation."""
import math
import os
import itertools
import json
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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from as2_msgs.msg import PlatformInfo, PlatformStatus
from rclpy.qos import qos_profile_sensor_data
from .facing import FacingError, FacingSession, facing_yaw
from uav_object_detection_interfaces.srv import DetectSnapshot
from uav_object_detection_interfaces.msg import ObjectDetectionFrame
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
        self._observation_mode=self.declare_parameter('observation_mode','snapshot').value
        if self._observation_mode not in ('snapshot','stream'):
            raise ValueError('unknown observation_mode')
        self._stream_query=self.declare_parameter('stream_query','person').value.strip().lower()
        self._stream_lock=threading.Lock()
        self._latest_frame=None
        self._stream_sequence=0
        self._stream_taken=0
        self._stream_dropped=0
        self._stream_subscription=None
        self.timing_sequence=itertools.count()
        self.timing_publisher=(self.create_publisher(String,'/diagnostics/vision_pipeline',100)
            if self.declare_parameter('timing_enabled',False).value else None)
        self.declare_parameter('snapshot_timeout', 1.0)
        self.declare_parameter('max_capture_age', 0.5)
        noise = float(self.declare_parameter('measurement_stddev_m', 0.0).value)
        if not math.isfinite(noise) or noise < 0:
            raise ValueError('measurement_stddev_m must be finite and nonnegative (0 preserves upstream)')
        self._max_prediction_time=float(self.declare_parameter('max_prediction_time',.5).value)
        if not math.isfinite(self._max_prediction_time) or not .5<=self._max_prediction_time<=2.:
            raise ValueError('max_prediction_time must be within .5..2 seconds')
        self._position_filter=self.declare_parameter('position_filter','none').value
        if self._position_filter not in ('none','robust_linear_v1'):
            raise ValueError('unknown position_filter')
        self.filter_publisher=self.create_publisher(String,'tools/target_tracking/measurement_filter',10)
        self._measurement_stddev = noise or None
        self._association_policy = self.declare_parameter('association_policy', 'upstream').value
        if self._association_policy not in ('upstream', 'innovation_covariance_v1', 'soft_innovation_v1'):
            raise ValueError('unknown association_policy')
        if self._association_policy != 'upstream' and self._measurement_stddev is None:
            raise ValueError('covariance association requires explicit measurement_stddev_m')
        for name in ('snapshot_timeout', 'max_capture_age'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        self.state_publisher = self.create_publisher(TargetState, 'tools/target_tracking/state', 10)
        self._navigation = None
        self._own = self._platform = None
        self._follow_blocked = False
        self._ekf_class = load_ekf()  # fail startup before accepting goals if provenance is unavailable
        self._stopping = threading.Event()
        self._slot = threading.Lock()
        self._pending = None
        if self._observation_mode=='stream':
            self._stream_subscription=self.create_subscription(ObjectDetectionFrame,
                'perception/object_detection/detections',self.receive_frame,1)
        self._generation = 0
        self._clock_epoch = 0
        self._jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=self._clock_jump)
        group = self._group = ReentrantCallbackGroup()
        self._client = self.create_client(DetectSnapshot, 'tools/object_detection/observe', callback_group=group)
        self._server = ActionServer(self, TrackObject, 'tools/target_tracking/track',
                                    execute_callback=self.execute, goal_callback=self.goal,
                                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                                    callback_group=group)

    def trace(self, stage, message=None, **details):
        publisher=getattr(self,'timing_publisher',None)
        if publisher is None:return
        header=getattr(message,'image_header',None)
        if header is None or (header.stamp.sec==0 and header.stamp.nanosec==0):
            header=getattr(message,'header',None)
        sample_ns=(header.stamp.sec*10**9+header.stamp.nanosec) if header is not None else None
        event=dict(stage=stage,sample_ns=sample_ns,mono_ns=time.monotonic_ns(),
            wall_ns=time.time_ns(),ros_ns=self.get_clock().now().nanoseconds,
            node=self.get_name(),pid=os.getpid(),seq=next(self.timing_sequence),**details)
        publisher.publish(String(data=json.dumps(event,separators=(',',':'))))

    def _ensure_facing_inputs(self):
        if self._navigation is not None:
            return
        from .navigation_transport import NavigationTransport
        self.create_subscription(PoseStamped, 'self_localization/pose',
            lambda msg:setattr(self, '_own', (msg, time.monotonic())), qos_profile_sensor_data)
        self.create_subscription(PlatformInfo, 'platform/info',
            lambda msg:setattr(self, '_platform', (msg, time.monotonic())), qos_profile_sensor_data)
        self._navigation = NavigationTransport(self, self._group)

    def _facing_own(self, state, now):
        wall = time.monotonic()
        for entry in (self._own, self._platform):
            if entry is None or not 0 <= wall-entry[1] <= .5 or not 0 <= now-stamp_value(entry[0].header.stamp) <= .5:
                raise FacingError('OWN_STATE_NOT_READY', 'fresh own pose and platform required for turning')
        pose, platform = self._own[0], self._platform[0]
        if pose.header.frame_id != state.header.frame_id:
            raise FacingError('FRAME_MISMATCH', 'own pose and target must share a fixed frame')
        if not (platform.connected and platform.armed and platform.offboard and platform.status.state==PlatformStatus.FLYING):
            raise FacingError('NOT_FLYING', 'follow=true requires an armed offboard flying platform')
        p = pose.pose.position
        return (p.x,p.y,p.z)

    def _clock_jump(self, jump):
        self._clock_epoch += 1
        self.get_logger().warning(
            f'ROS clock generation {self._clock_epoch}: change={jump.clock_change}, delta_ns={jump.delta.nanoseconds}')

    def receive_frame(self, frame):
        self.trace('tracking_receive',frame)
        with self._stream_lock:
            if self._latest_frame is not None and self._stream_sequence>self._stream_taken:
                self._stream_dropped+=1
            self._stream_sequence+=1
            self._latest_frame=(self._stream_sequence,frame,time.monotonic(),self._clock_epoch)

    def goal(self, request):
        if self._observation_mode=='stream' and request.query.strip().lower()!=self._stream_query:
            return GoalResponse.REJECT
        if request.follow and self._follow_blocked:
            return GoalResponse.REJECT
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
        tracker = Tracker(self._ekf_class, measurement_stddev_m=self._measurement_stddev, association_policy=self._association_policy, position_filter=self._position_filter,max_prediction_time=self._max_prediction_time)
        state = TargetState(session_id=session, generation=generation, status='unavailable')
        capture = 0
        attempt = 0
        deadline = time.monotonic() + handle.request.timeout
        last_clock = self.get_clock().now().nanoseconds * 1e-9
        reason = 'No usable observations'
        code = 'TIMEOUT'
        stream_started=last_clock
        stream_sequence=0
        stream_identity=None
        facing = None
        stop_confirmed = True
        try:
            if handle.request.follow:
                self._ensure_facing_inputs()
                facing = FacingSession(self._navigation)
            while rclpy.ok() and not self._stopping.is_set():
                now = self.get_clock().now().nanoseconds * 1e-9
                if handle.is_cancel_requested:
                    code = 'CANCELED'; reason = 'Canceled by caller'; break
                if self._clock_epoch != epoch or now < last_clock:
                    code = 'CLOCK_RESET'; reason = 'ROS clock generation changed or clock moved backward'; break
                last_clock = now
                if time.monotonic() >= deadline:
                    reason = 'Observation window elapsed'; break
                # Do not release transport occupancy on cancel/timeout: a late
                # response must drain before a subsequent session issues a call.
                if self._observation_mode=='stream':
                    with self._stream_lock:
                        latest=self._latest_frame
                        if latest is not None:self._stream_taken=latest[0]
                    if latest is not None and latest[0]>stream_sequence:
                        stream_sequence,frame,received,frame_epoch=latest
                        self.trace('tracking_take',frame,sequence=stream_sequence,
                            queue_ms=(time.monotonic()-received)*1000)
                        stamp=stamp_value(frame.header.stamp)
                        identity=(frame.tool_generation,frame.worker_generation,frame.model_version)
                        fresh=(frame_epoch==epoch and stamp>=stream_started and
                            0<=now-stamp<=float(self.get_parameter('max_capture_age').value) and
                            time.monotonic()-received<=float(self.get_parameter('max_capture_age').value))
                        if fresh:
                            if stream_identity is not None and stream_identity!=identity:
                                code='OBSERVATION_GENERATION_CHANGED'
                                reason='stream producer identity changed; restart session'
                                break
                            stream_identity=identity
                            try:
                                points=[[d.position.x,d.position.y,d.position.z]for d in frame.detections if d.position_valid]
                                if not frame.valid and not points:
                                    raise ValueError(frame.error or 'stream frame has no valid 3D positions')
                                reason=tracker.observe(points,stamp,frame.header.frame_id)
                                self.filter_publisher.publish(String(data=json.dumps(tracker.measurement_audit)))
                                capture+=1
                                self.trace('tracking_processed',frame,position_count=len(points),
                                    tracker_status=reason,filter_status=tracker.measurement_audit['status'],
                                    observation_ns=int(tracker.observation_stamp*1e9) if tracker.observation_stamp is not None else None)
                            except ValueError as error:
                                reason=str(error)
                                self.trace('tracking_reject',frame,reason=reason)
                        else:
                            self.trace('tracking_reject',frame,reason='stale_or_session_epoch')
                            reason='stream result predates session, is stale or has wrong clock epoch'
                if self._observation_mode=='snapshot' and self._pending is not None and self._pending.done():
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
                                self.filter_publisher.publish(String(data=json.dumps(tracker.measurement_audit)))
                                capture = self._pending_capture
                                reason = observed_status
                            except ValueError as exc:
                                reason = str(exc)
                        else:
                            reason = response.reason or 'unavailable or stale capture'
                if self._observation_mode=='snapshot' and self._pending is None and self._client.service_is_ready():
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
                if facing is not None:
                    if state.valid:
                        # Wait briefly for new subscriptions, without issuing motion.
                        if facing.started or (self._own is not None and self._platform is not None):
                            own = self._facing_own(state, now)
                            target = (state.position.x,state.position.y,state.position.z)
                            facing.step(own, state.header.frame_id, facing_yaw(own,target),
                                        max(.01, deadline-time.monotonic()))
                    elif facing.started:
                        raise FacingError('TARGET_LOST', 'target expired; stop turning')
                self.state_publisher.publish(state)
                handle.publish_feedback(TrackObject.Feedback(state=state))
                time.sleep(.02)
            else:
                code = 'SHUTDOWN'
        except Exception as exc:
            code = exc.code if isinstance(exc, FacingError) else 'ERROR'
            reason = str(exc)
        finally:
            if facing is not None:
                stop_confirmed = facing.close()
                if not stop_confirmed:
                    self._follow_blocked = True
                    code = 'STOP_UNCONFIRMED'
                    reason += '; owned navigation stop unconfirmed'
            self._generation += 1  # invalidate outstanding calls before releasing slot
            self._slot.release()
        state.valid = False
        state.reason = reason
        self.state_publisher.publish(state)
        # Terminal status follows Navigation cleanup, never precedes it.
        if code == 'TIMEOUT':
            handle.succeed()
        elif code == 'CANCELED':
            handle.canceled()
        elif handle.is_active:
            handle.abort()
        return TrackObject.Result(code=code, reason=reason, stop_confirmed=stop_confirmed, state=state)


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
