"""FlowPilot planning lifecycle, derived from the YOPO reference planner (MIT).

Independently maintained so a new model never needs edits to the YOPO package.
Motion references are exclusively owned by Behavior.
"""
import copy
from collections import deque
import json
import math
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from as2_msgs.msg import ControllerInfo, ControlMode
from uav_inference_interfaces.srv import InferFlowPilot
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal
from uav_planner_interfaces.msg import TimedTrajectoryPoint
from .core import Timeline
from .curve import brake_segment


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def set_time(stamp, value):
    ns = round(value * 1e9)
    stamp.sec, stamp.nanosec = divmod(ns, 1000000000)


def vector(msg):
    return np.array([msg.x, msg.y, msg.z])


def assign(msg, values):
    msg.x, msg.y, msg.z = map(float, values)


class FlowPilotLifecycle(Node):
    def __init__(self, **kwargs):
        super().__init__('flowpilot_planner', **kwargs)
        def param(name, value):
            return self.declare_parameter(name, value).value
        self.vehicle = param('vehicle_id', 'drone0')
        self.frame = param('fixed_frame', self.vehicle + '/odom')
        self.optical_frame = param('optical_frame', self.vehicle + '/camera/front_center_optical')
        self.aligned_frame = self.vehicle + '/flowpilot_aligned'
        self.lead = param('handoff_lead_s', .2)
        self.timeout = param('inference_timeout_s', .1)
        self.reserve = param('submission_reserve_s', .05)
        self.age = param('max_observation_age_s', .3)
        self.skew = param('max_observation_skew_s', .05)
        self.max_speed = param('max_speed', 6.)
        self.command_speed = param('command_speed_mps', self.max_speed)
        self.arrival_margin = param('arrival_brake_margin_m', 1.25)
        self.initial_plan_timeout = param('initial_plan_timeout_s', 1.)
        self.max_yaw_rate = param('max_yaw_rate_rad_s', math.pi / 2)
        self.plan_acceleration_cap = param('plan_acceleration_cap_mps2', 1.0)
        self.brake_acceleration = param('brake_acceleration_mps2', 1.0)
        self.replan_remaining_min = param('replan_remaining_s', .8)
        self.track_altitude_enabled = param('goal_altitude_tracking', True)
        self.altitude_rate = param('goal_altitude_rate_mps', .5)
        self.altitude_time_constant = param('goal_altitude_time_constant_s', 3.)
        self.altitude_climb_margin = param('goal_altitude_climb_margin_m', .5)
        box = param('plan_world_box', [-6., 6., -5., 16., .5, 4.])
        if (not isinstance(box, list) or len(box) != 6 or
                not all(isinstance(v, float) and math.isfinite(v) for v in box) or
                not all(box[i] < box[i+1] for i in (0, 2, 4))):
            raise ValueError('plan_world_box must be 6 finite bounds [x0,x1,y0,y1,z0,z1]')
        self.plan_lo = np.array([box[0], box[2], box[4]])
        self.plan_hi = np.array([box[1], box[3], box[5]])
        self.handoff_limits = dict(position=param('handoff_position_tolerance_m', .15),
                                   velocity=param('handoff_velocity_tolerance_mps', .5),
                                   acceleration=param('handoff_acceleration_tolerance_mps2', .5),
                                   yaw=param('handoff_yaw_tolerance_rad', .15))
        self.camera_pitch = param('camera_pitch_deg', 0.)
        # Fail closed unless deployment explicitly identifies the measurement model.
        self.depth_model = param('depth_model', 'unverified')
        if not all(math.isfinite(x) and x > 0 for x in
                   (self.lead, self.timeout, self.reserve, self.age, self.skew, self.max_speed)):
            raise ValueError('timing and speed parameters must be finite and positive')
        if self.lead < self.timeout + self.reserve or self.lead > .3:
            raise ValueError('handoff lead must cover timeout+reserve and fit adapter future bound')
        if not math.isfinite(self.camera_pitch):
            raise ValueError('camera pitch must be finite')
        if not (math.isfinite(self.arrival_margin) and 0 <= self.arrival_margin < 5):
            raise ValueError('arrival brake margin must be finite in [0,5)')
        if (not all(math.isfinite(v) and v > 0 for v in (self.altitude_rate, self.altitude_time_constant))
                or not math.isfinite(self.altitude_climb_margin) or self.altitude_climb_margin < 0):
            raise ValueError('altitude rate/time constant must be finite positive; climb margin finite nonnegative')
        limits = (self.command_speed, self.initial_plan_timeout, self.max_yaw_rate,
                  self.plan_acceleration_cap, self.brake_acceleration, self.replan_remaining_min,
                  *self.handoff_limits.values())
        if not all(math.isfinite(value) and value > 0 for value in limits) or self.command_speed > 10:
            raise ValueError('command speed must be in (0,10]; deadlines, yaw rate and tolerances must be positive')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rotation_bc = Rotation.from_euler('Y', self.camera_pitch, degrees=True).as_matrix()
        self.pose = self.twist = self.depth = self.info = None
        self.pose_history, self.twist_history = deque(maxlen=200), deque(maxlen=200)
        self.pending_twists = deque(maxlen=200)
        self.goal = self.queued_goal = None
        self.timeline = Timeline()
        self.generation = 0
        self.inference = self.execution = None
        self.goal_handle = None
        self.failures = 0
        self.mode = None
        self.mode_at = 0.
        self.state = 'IDLE'
        self.reason = ''
        self.last_clock = None
        self.last_image = None
        self.sync_missing_since = None
        self.stop_future = None
        self.stop_ack = False
        self.settled_since = None
        self.status = self.create_publisher(String, 'flowpilot/status', 10)
        self.events = self.create_publisher(String, 'flowpilot/events', 100)
        self.create_subscription(PoseStamped, 'self_localization/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, 'self_localization/twist', self.on_twist, qos_profile_sensor_data)
        self.create_subscription(Image, 'flowpilot/depth', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'flowpilot/camera_info', self.on_info, qos_profile_sensor_data)
        self.navigation = None
        if param('navigation_action', True):
            from .navigation import FlowPilotNavigation
            self.navigation = FlowPilotNavigation(self)
        self.create_subscription(PoseStamped, 'flowpilot/goal', self.on_goal, 1)
        self.create_subscription(ControllerInfo, 'controller/info', self.on_mode, qos_profile_sensor_data)
        self.create_service(Trigger, 'flowpilot/cancel', self.cancel)
        self.infer_client = self.create_client(InferFlowPilot, 'inference/flowpilot/infer')
        self.action = ActionClient(self, FollowTimedPath, 'follow_timed_path')
        self.modify = self.create_client(FollowTimedPath_SendGoal, 'follow_timed_path/_behavior/modify')
        self.stopper = self.create_client(Trigger, 'follow_timed_path/_behavior/stop')
        self.create_timer(.02, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def fixed_transform(self, header):
        if header.frame_id == self.frame:
            return np.eye(3), np.zeros(3)
        if header.frame_id not in ('earth', self.vehicle+'/base_link'):
            raise ValueError('unsupported localization frame')
        transform = self.tf_buffer.lookup_transform(
            self.frame, header.frame_id, Time.from_msg(header.stamp)).transform
        q = transform.rotation
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), vector(transform.translation)

    def on_pose(self, msg):
        if self.pose is not None and seconds(msg.header.stamp) <= seconds(self.pose.header.stamp):
            return
        try:
            rotation, translation = self.fixed_transform(msg.header)
            converted = copy.deepcopy(msg)
            assign(converted.pose.position, rotation @ vector(msg.pose.position)+translation)
            q = msg.pose.orientation
            orientation = Rotation.from_matrix(rotation) * Rotation.from_quat([q.x, q.y, q.z, q.w])
            converted.pose.orientation.x, converted.pose.orientation.y, converted.pose.orientation.z, converted.pose.orientation.w = map(float, orientation.as_quat())
            converted.header.frame_id = self.frame
            self.pose = converted
            self.pose_history.append(converted)
        except (TransformException, ValueError):
            pass

    def on_twist(self, msg):
        if not self.convert_twist(msg):
            self.pending_twists.append(msg)

    def convert_twist(self, msg):
        if self.twist is not None and seconds(msg.header.stamp) <= seconds(self.twist.header.stamp):
            return True
        try:
            if msg.header.frame_id == self.vehicle+'/base_link':
                # Native AS2 publishTwist emits the authority pose with the EXACT
                # same measurement stamp. Use that body attitude, not a delayed
                # duplicate dynamic TF stream or a latest-time approximation.
                pose = next((p for p in reversed(self.pose_history)
                             if p.header.stamp == msg.header.stamp), None)
                if pose is None:
                    return False
                q = pose.pose.orientation
                rotation = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            else:
                rotation, _ = self.fixed_transform(msg.header)
            converted = copy.deepcopy(msg)
            assign(converted.twist.linear, rotation @ vector(msg.twist.linear))
            assign(converted.twist.angular, rotation @ vector(msg.twist.angular))
            converted.header.frame_id = self.frame
            self.twist = converted
            self.twist_history.append(converted)
            return True
        except (TransformException, ValueError):
            return False

    def on_depth(self, msg):
        if (msg.header.frame_id == self.optical_frame and
                (self.depth is None or seconds(msg.header.stamp) > seconds(self.depth.header.stamp))):
            self.depth = msg  # latest-only mailbox

    def on_info(self, msg):
        if msg.header.frame_id == self.optical_frame:
            self.info = msg

    def on_mode(self, msg):
        self.mode = msg.input_control_mode.control_mode
        self.mode_at = self.now_s()

    def geometry_ready(self):
        if self.depth_model != 'axial' or self.info is None or self.depth is None:
            return False
        try:
            transform = self.tf_buffer.lookup_transform(
                self.vehicle+'/base_link', self.optical_frame, Time.from_msg(self.depth.header.stamp)).transform
            q = transform.rotation
            actual = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            aligned_optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
            if (np.linalg.norm(vector(transform.translation)) > 1e-6 or
                    not np.allclose(actual, self.rotation_bc @ aligned_optical, atol=1e-6)):
                return False
        except (TransformException, ValueError):
            return False
        c = self.info
        if c.width != self.depth.width or c.height != self.depth.height or c.k[0] <= 0 or c.k[4] <= 0:
            return False
        hfov = math.degrees(2*math.atan(c.width/(2*c.k[0])))
        vfov = math.degrees(2*math.atan(c.height/(2*c.k[4])))
        return (abs(hfov-90) < 1 and abs(vfov-60) < 1 and
                abs(c.k[2]-c.width/2) < 1 and abs(c.k[5]-c.height/2) < 1 and
                all(abs(d) < 1e-9 for d in c.d))

    def observation_pair(self):
        if self.depth is None or not self.pose_history or not self.twist_history:
            return None
        epoch = seconds(self.depth.header.stamp)
        # AS2 authority publishes Pose and Twist for the same measurement epoch.
        twists = {(msg.header.stamp.sec, msg.header.stamp.nanosec): msg for msg in self.twist_history}
        for pose in sorted(self.pose_history, key=lambda msg: abs(seconds(msg.header.stamp)-epoch)):
            key = (pose.header.stamp.sec, pose.header.stamp.nanosec)
            if key in twists and abs(seconds(pose.header.stamp)-epoch) <= self.skew:
                return pose, twists[key]
        return None

    def inputs_fresh(self, now):
        if self.pose is None or self.twist is None or not self.geometry_ready():
            return False
        stamps = [seconds(m.header.stamp) for m in (self.pose, self.twist, self.depth)]
        values = np.r_[vector(self.pose.pose.position), vector(self.twist.twist.linear)]
        # Synchronize against the bounded history, not the newest pose on every timer tick.
        return np.isfinite(values).all() and all(0 <= now-t <= self.age for t in stamps)

    def observations_ready(self, now):
        return self.inputs_fresh(now) and self.observation_pair() is not None

    def on_goal(self, msg):
        if self.navigation is not None:
            self.get_logger().error('Use the public navigate_to_pose Action')
            return
        if msg.header.frame_id != self.frame or not np.isfinite(vector(msg.pose.position)).all():
            self.get_logger().error('Goal requires finite fixed ENU coordinates')
            return
        if self.state == 'FAULT':
            self.get_logger().error('Stop recovery required before restarting planner')
            return
        if self.state in ('RUNNING', 'STOPPING'):
            self.queued_goal = copy.deepcopy(msg)
            self.stop('GOAL_CHANGED')
            return
        if self.inputs_fresh(self.now_s()) and self.ready() and self.observation_pair() is None:
            self.queued_goal = copy.deepcopy(msg)
            return
        if not self.observations_ready(self.now_s()) or not self.ready():
            self.publish_status('NOT_READY', 'sensor geometry, observations or services unavailable')
            return
        self.begin(msg)

    def ready(self):
        return (self.infer_client.service_is_ready() and self.action.server_is_ready()
                and self.modify.service_is_ready() and self.stopper.service_is_ready())

    def begin(self, msg):
        self.generation += 1
        self.queued_goal = None
        self.goal = vector(msg.pose.position)
        self.task = 'flowpilot-' + uuid.uuid4().hex
        self.timeline = Timeline()
        self.goal_handle = None
        self.last_image = None
        self.failures = 0
        self.state = 'RUNNING'
        self.first_plan_deadline = time.monotonic() + self.initial_plan_timeout
        self.sync_missing_since = None
        self.reason = ''

    def cancel(self, request, response):
        if self.navigation is not None:
            response.success = False
            response.message = 'Cancel through the owning navigation Action'
            return response
        self.queued_goal = None
        self.stop('CANCELED')
        response.success = self.state != 'FAULT'
        response.message = 'stop requested; await flowpilot/status STOPPED and speed gate'
        return response

    def publish_status(self, state=None, reason=None):
        self.status.publish(String(data=json.dumps(dict(
            state=state or self.state, reason=self.reason if reason is None else reason,
            ready=self.observations_ready(self.now_s()) and self.ready(),
            task_id=getattr(self, 'task', ''), control_mode=self.mode,
            stop_ack=self.stop_ack, input_ages={name:self.now_s()-seconds(msg.header.stamp)
                for name,msg in [('pose',self.pose),('twist',self.twist),('depth',self.depth)] if msg is not None},
            synchronized=self.observation_pair() is not None, pending_twist_transforms=len(self.pending_twists),
            revision=self.timeline.revision, effective_revision=self.timeline.effective))))

    def stop(self, reason):
        if self.state in ('IDLE', 'STOPPED', 'STOPPING', 'FAULT'):
            return
        self.generation += 1  # invalidates inference and execution responses
        self.discard_inference()
        self.state, self.reason = 'STOPPING', reason
        self.stop_started = time.monotonic()
        self.stop_ack = False
        self.settled_since = None
        self.stop_future = None
        if not self.brake_to_stop(self.now_s()):
            self.request_stop()

    def brake_to_stop(self, now):
        """Decelerate to rest through the behavior's own final-segment path.

        The stop service alone sends a single hover reference which leaves the
        controller in trajectory mode holding the last setpoint, so the drone
        never settles and the stop handshake times out. A final braking
        segment instead ends in the behavior's position-command settle stream
        (controller mode switches to POSITION), which stop_tick observes.
        """
        if self.timeline.latest is None or self.execution is not None:
            return False
        try:
            epoch = round((now + self.lead) * 1e9) / 1e9
            reference = self.timeline.reference(epoch)
            curve, duration = brake_segment(reference, epoch, self.brake_acceleration,
                                            self.max_yaw_rate)
            self.submit(curve, now, final=True)
            self.events.publish(String(data=json.dumps(dict(
                event='brake', task=self.task, start=curve.start, duration=round(duration, 3),
                speed=round(float(np.linalg.norm(reference[0][1])), 3)))))
            return True
        except Exception as error:
            self.get_logger().warning('braking stop unavailable: ' + str(error))
            return False

    def discard_inference(self):
        if self.inference is not None:
            future = self.inference['future']
            self.infer_client.remove_pending_request(future)
            future.cancel()
            self.inference = None

    def request_stop(self):
        if self.stopper.service_is_ready():
            self.stop_future = self.stopper.call_async(Trigger.Request())

    def stop_tick(self, now):
        # Reissue after any old submission completes: stop must follow late accepts.
        if self.stop_future is not None and self.stop_future.done():
            try:
                self.stop_ack = self.stop_future.result().success
            except Exception:
                self.stop_ack = False
            self.stop_future = None
        unresolved = self.execution is not None and not self.execution['future'].done()
        if self.stop_ack and not unresolved:
            if self.execution is not None:
                self.execution = None
                self.stop_ack = False
                self.request_stop()
                return
        fresh = (self.twist is not None and self.pose is not None and
                 all(0 <= now-seconds(m.header.stamp) <= self.age for m in (self.pose, self.twist)))
        mode_ok = self.mode in (ControlMode.HOVER, ControlMode.POSITION) and 0 <= now-self.mode_at <= .5
        # The stop service confirms hover submission; controller mode and motion confirm effect.
        if self.stop_ack and not unresolved and fresh and mode_ok and np.linalg.norm(vector(self.twist.twist.linear)) < .2:
            if self.settled_since is None:
                self.settled_since = now
            if now-self.settled_since >= 1 and self.inference is None:
                self.state = 'STOPPED'
                self.timeline = Timeline()
                if self.queued_goal is not None and self.observations_ready(now) and self.ready():
                    goal, self.queued_goal = self.queued_goal, None
                    self.begin(goal)
        else:
            self.settled_since = None
        if self.state == 'STOPPING' and time.monotonic()-self.stop_started > 10:
            self.state, self.reason = 'FAULT', 'STOP_TIMEOUT:' + self.reason

    def tick(self):
        now = self.now_s()
        pending, self.pending_twists = self.pending_twists, deque(maxlen=200)
        for measurement in pending:
            if abs(now-seconds(measurement.header.stamp)) <= self.age and not self.convert_twist(measurement):
                self.pending_twists.append(measurement)
        if self.last_clock is not None and now < self.last_clock:
            self.stop('CLOCK_RESET')
            self.pose_history.clear(); self.twist_history.clear(); self.pending_twists.clear()
            self.pose = self.twist = self.depth = None
            self.last_image = None
            self.queued_goal = None
        self.last_clock = now
        # Removing a local ROS request cannot cancel the worker's GPU computation.
        # The worker keeps its busy slot until that computation actually finishes.
        if self.inference is not None:
            context = self.inference
            if context['future'].done():
                self.inference = None
                if context['generation'] == self.generation and not context['expired']:
                    self.inferred(context, now)
            elif not context['expired'] and time.monotonic()-context['sent'] > self.timeout:
                context['expired'] = True
                self.stop('INFERENCE_TIMEOUT')
        if self.state == 'STOPPING':
            self.stop_tick(now)
        if self.state in ('IDLE', 'STOPPED') and self.queued_goal is not None and self.observations_ready(now) and self.ready():
            goal, self.queued_goal = self.queued_goal, None
            self.begin(goal)
        if self.state == 'RUNNING':
            if not self.timeline.revision and time.monotonic() > self.first_plan_deadline:
                self.stop('INITIAL_PLAN_TIMEOUT')
            elif not self.inputs_fresh(now):
                self.stop('STALE_OR_INVALID_INPUT')
            # Stop inside the 5 m arrival radius by the braking margin so the
            # settled position, not the moving trigger point, meets the radius.
            elif np.linalg.norm(vector(self.pose.pose.position)-self.goal) < 5 - self.arrival_margin:
                self.stop('ARRIVED_5M')
            elif self.timeline.latest and self.timeline.latest.end-now < self.lead+self.reserve:
                self.stop('COVERAGE_EXHAUSTED')
            elif self.execution is not None and now >= self.execution['curve'].start-self.reserve/2:
                self.stop('EXECUTION_ACK_TIMEOUT')
            elif self.observation_pair() is None:
                if self.sync_missing_since is None:
                    self.sync_missing_since = time.monotonic()
                elif time.monotonic()-self.sync_missing_since > self.age:
                    self.stop('UNSYNCHRONIZED_INPUT')
            elif self.can_infer():
                self.sync_missing_since = None
                self.request_inference(now)
            else:
                self.sync_missing_since = None
        self.publish_status()

    def can_infer(self):
        if self.inference is not None or self.execution is not None or not self.ready() or self.observation_pair() is None:
            return False
        if self.timeline.revision and (self.timeline.effective != self.timeline.revision or
                self.mode != ControlMode.TRAJECTORY or self.now_s()-self.mode_at > .5):
            return False
        if self.timeline.latest is not None:
            # Replace a segment only once most of its horizon is consumed: the
            # model's plan starts with an acceleration ramp, so re-planning
            # every cycle restarts the ramp and the commanded stream never
            # accelerates (observed 0.03 m/s crawl at 0.45 s cadence). Executing
            # most of each plan lets speed compound across revisions.
            remaining = self.timeline.latest.end - self.now_s()
            threshold = max(self.replan_remaining_min,
                            .45 * (self.timeline.latest.end - self.timeline.latest.start))
            if remaining > threshold:
                return False
        return seconds(self.depth.header.stamp) != self.last_image

    def failure(self, reason):
        if self.state != 'RUNNING':
            return
        self.failures += 1
        self.get_logger().warning(reason)
        if self.failures >= 3:
            self.stop(reason)

    def request_inference(self, now):
        raise NotImplementedError("model-specific inference request")

    def inferred(self, context, now):
        raise NotImplementedError("model-specific inference result")

    def curve_event(self, curve):
        return dict(duration=curve.end - curve.start, observation=curve.observation,
                    scale=round(getattr(curve, 'scale', 1.), 3),
                    bernstein_world=curve.coefficients[0].tolist(), times=curve.times.tolist())

    def submit(self, curve, now, final=False):
        revision = self.timeline.revision + 1
        goal = FollowTimedPath.Goal()
        path = goal.path
        path.interface_version, path.task_id, path.vehicle_id = 1, self.task, self.vehicle
        path.revision, path.frame_id, path.valid_fields = revision, self.frame, 7
        path.max_speed, path.is_final = float(self.max_speed), bool(final)
        set_time(path.generated_at, now)
        set_time(path.valid_from, curve.start)
        for t in curve.times:
            point = TimedTrajectoryPoint()
            pva, yaw = curve.sample(curve.start+t)
            set_time(point.time_from_start, t)
            assign(point.position, pva[0]); assign(point.velocity, pva[1]); assign(point.acceleration, pva[2])
            point.yaw = float(yaw)
            path.points.append(point)
        if curve.start-self.now_s() < self.reserve:
            raise ValueError('serialization missed submission budget')
        self.timeline.propose(curve)
        generation = self.generation
        try:
            if revision == 1:
                future = self.action.send_goal_async(goal, feedback_callback=lambda m: self.feedback(m, generation))
            else:
                request = FollowTimedPath_SendGoal.Request()
                request.goal = goal
                future = self.modify.call_async(request)
        except Exception:
            self.timeline.acknowledge(False)
            raise
        self.events.publish(String(data=json.dumps(dict(
            event='submit', task=self.task, revision=revision, generated=now,
            start=curve.start, final=bool(final), **self.curve_event(curve)))))
        self.execution = dict(future=future, curve=curve)
        future.add_done_callback(lambda f: self.accepted(f, revision, generation))

    def accepted(self, future, revision, generation):
        try:
            response = future.result()
            if generation != self.generation:
                if revision == 1 and response.accepted:
                    response.cancel_goal_async()
                return
            self.execution = None
            accepted = response.accepted
            self.timeline.acknowledge(accepted)
            self.events.publish(String(data=json.dumps(dict(event='ack', task=self.task, revision=revision, accepted=accepted, at=self.now_s()))))
            if not accepted:
                self.failure('EXECUTION_REJECTED')
                return
            self.failures = 0
            if revision == 1:
                self.goal_handle = response
                response.get_result_async().add_done_callback(lambda f: self.result(f, generation))
        except Exception as error:
            if generation == self.generation:
                self.stop('EXECUTION_UNCERTAIN:' + str(error))

    def feedback(self, message, generation):
        if generation != self.generation or self.state != 'RUNNING':
            return
        feedback = message.feedback
        if feedback.task_id == self.task:
            self.timeline.feedback(feedback.revision, feedback.elapsed, self.now_s(), feedback.waiting_for_start)

    def result(self, future, generation):
        if self.state == 'STOPPING':
            # The goal (typically a braking final segment) has ended in the
            # behavior's settle stream; confirm the stop so stop_tick can
            # observe the controller mode and speed decay.
            self.request_stop()
            return
        if generation == self.generation and self.state == 'RUNNING':
            try:
                reason = future.result().result.reason
            except Exception as error:
                reason = str(error)
            self.stop('EXECUTION_ENDED:' + reason)
