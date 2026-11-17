"""Task-call wrapper around the existing SUPER adapter, not a planner.

Arrival checks are the PC Demo's existing 1.5 m / 0.3 m/s / 2 s checks.
The adapter's polynomial conversion and FollowTimedPath lifecycle stay intact.
"""
import copy
import math
import time

from uav_motion_resource.lease import MotionLease

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
from uav_planner_interfaces.action import NavigateToPose
from uav_planner_interfaces.srv import UpdateNavigationTarget


class SuperNavigation:
    def __init__(self, adapter):
        self.node = adapter
        self.reserved = False
        self.motion = MotionLease(adapter.get_namespace())
        self.motion_stopped = False
        self.handle = None
        self.finished = None
        self.odom = None
        self.stable_since = None
        self.stop_started = None
        self.outcome = None
        self.phase = 'idle'
        self.started = 0.
        self.timeout = 300.
        self.target_revision = 0
        self.holding = False
        self.resume_after_stop = False
        self.last_target = 0.
        self.publisher = adapter.create_publisher(PoseStamped, '/goal_pose', 1)
        self.subscription = adapter.create_subscription(
            Odometry, 'sensor_measurements/odom', self.on_odom,
            qos_profile_sensor_data)
        self.server = ActionServer(adapter, NavigateToPose, 'navigate_to_pose',
                                   execute_callback=self.execute,
                                   goal_callback=self.on_goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())

        self.update_service = adapter.create_service(
            UpdateNavigationTarget, 'navigate_to_pose/update_target', self.update_target)

    def on_odom(self, message):
        if message.header.frame_id == self.node.frame:
            self.odom = message

    def on_goal(self, request):
        if self.reserved or self.node.enabled:
            return GoalResponse.REJECT
        if self.node.client and not self.node.client.quiescent:
            return GoalResponse.REJECT
        if not self.motion.acquire('navigation.super'):
            return GoalResponse.REJECT
        self.motion_stopped = False
        self.reserved = True
        return GoalResponse.ACCEPT

    def validate(self, request):
        p = request.target.pose.position
        if request.target.header.frame_id != self.node.frame:
            return 'INVALID_GOAL', 'Target frame must be ' + self.node.frame
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return 'INVALID_GOAL', 'Target position must be finite'
        if request.orientation_mode == request.TARGET_YAW:
            if not self.node.get_parameter('target_yaw_enabled').value:
                return 'UNSUPPORTED', 'target_yaw_enabled is false'
            q = request.target.pose.orientation
            norm = sum(v*v for v in (q.x,q.y,q.z,q.w))
            if not math.isfinite(norm) or abs(norm-1.) > 1e-3:
                return 'INVALID_GOAL', 'Target yaw requires a finite unit quaternion'
            if abs(q.x)>1e-6 or abs(q.y)>1e-6:
                return 'UNSUPPORTED', 'TARGET_YAW requires a yaw-only quaternion'
        elif request.orientation_mode not in (request.UNCONSTRAINED, request.KEEP_YAW):
            return 'UNSUPPORTED', 'Only target yaw, not full attitude, is supported'
        if request.arrival_behavior not in (request.COMPLETE, request.HOLD):
            return 'INVALID_GOAL', 'Unknown arrival behavior'
        if request.arrival_behavior == request.HOLD and not request.use_target_timeout:
            return 'INVALID_GOAL', 'HOLD requires a target stream timeout'
        for name in ('max_speed', 'position_tolerance', 'timeout', 'target_timeout'):
            value = getattr(request, name)
            if getattr(request, 'use_' + name) and (not math.isfinite(value) or value <= 0):
                return 'INVALID_GOAL', name + ' must be finite and positive'
        if request.use_max_speed and not math.isclose(
                request.max_speed, self.node.get_parameter('max_speed').value):
            return 'UNSUPPORTED', 'Per-task speed changes are not supported; use the deployed speed'
        if self.odom is None or not -.05 <= self.node.now_seconds() - self.node.pose_stamp <= .5:
            return 'NOT_READY', 'Fresh local odometry required'
        if self.publisher.get_subscription_count() == 0:
            return 'NOT_READY', 'SUPER goal subscriber unavailable'
        return None

    async def execute(self, handle):
        self.handle = handle
        self.finished = Future()
        self.stable_since = self.stop_started = None
        self.outcome = None
        self.phase = 'preparing'
        self.started = self.last_target = self.node.now_seconds()
        self.target_revision = 1
        self.holding = self.resume_after_stop = False
        self.target = copy.deepcopy(handle.request.target)
        request = handle.request
        try:
            error = self.validate(request)
            if error:
                return self.finish(False, *error, stopped=True)
            if handle.is_cancel_requested:
                return self.finish(False, 'CANCELED', 'Canceled before execution', stopped=True)
            p = request.target.pose.position
            self.node.goal = [p.x, p.y, p.z]
            self.node.navigation_tolerance = (request.position_tolerance
                                             if request.use_position_tolerance else self.node.get_parameter('goal_tolerance').value)
            self.timeout = request.timeout if request.use_timeout else 300.
            response = self.node.enable(SetBool.Request(data=True), SetBool.Response(), internal=True)
            if not response.success:
                return self.finish(False, 'NOT_READY', response.message, stopped=True)
            self.publish_target()
            self.phase = 'planning'
            return await self.finished
        except Exception as exc:
            self.outcome = (False, 'INTERNAL_ERROR', str(exc))
            self.begin_stop()
            return await self.finished
        finally:
            self.motion.release(self.motion_stopped)
            self.handle = None
            self.reserved = False
            self.phase = 'idle'

    def publish_target(self):
        target = copy.deepcopy(self.target)
        if self.handle.request.orientation_mode != self.handle.request.TARGET_YAW:
            target.pose.orientation = copy.deepcopy(self.odom.pose.pose.orientation)
        target.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(target)

    def handoff_final_coverage(self, *, completed=False):
        # A final segment may have completed before a later target's plan is
        # ready. It is not a rolling replacement when their epochs don't overlap.
        h=self.handle
        if (not self.reserved or h is None or h.is_cancel_requested or
            self.outcome is not None or self.stop_started is not None or
            (h.request.arrival_behavior != h.request.HOLD and not completed) or
            (completed and (self.node.client is None or not self.node.client.complete))):
            return False
        self.resume_after_stop=True
        self.begin_stop()
        return True

    def restart_execution(self):
        # Reacquire execution only after the old FollowTimedPath stop gate.
        response = self.node.enable(SetBool.Request(data=True), SetBool.Response(), internal=True)
        if not response.success:
            self.outcome = (False, 'NOT_READY', response.message)
            self.begin_stop()
            return
        self.holding = self.resume_after_stop = False
        self.stable_since = None
        self.publish_target()
        self.phase = 'planning'

    def update_target(self, request, response):
        response.target_revision = self.target_revision
        h = self.handle
        error = None
        if h is None or self.finished is None or self.finished.done():
            error = ('NO_ACTIVE_TASK', 'No active navigation task')
        elif request.goal_id != h.goal_id:
            error = ('NOT_OWNER', 'Goal UUID does not own this task')
        elif h.is_cancel_requested or self.outcome is not None or (self.stop_started is not None and not self.resume_after_stop):
            error = ('STOPPING', 'Navigation is stopping')
        elif self.node.now_seconds() < self.started or self.node.now_seconds() - self.started > self.timeout:
            error = ('TIMEOUT', 'Navigation task deadline expired')
        elif h.request.use_target_timeout and self.node.now_seconds() - self.last_target > h.request.target_timeout:
            error = ('TARGET_TIMEOUT', 'Navigation target stream expired')
        elif request.target_revision <= self.target_revision:
            error = ('STALE_TARGET', 'Target revision must increase')
        elif request.orientation_mode != h.request.orientation_mode:
            error = ('UNSUPPORTED', 'Orientation mode cannot change during a task')
        else:
            candidate = copy.deepcopy(h.request)
            candidate.target = request.target
            error = self.validate(candidate)
        if error:
            response.accepted = False
            response.code, response.reason = error
            return response
        p = request.target.pose.position
        changed = list(self.node.goal) != [p.x, p.y, p.z]
        if h.request.orientation_mode == h.request.TARGET_YAW:
            a,b = self.target.pose.orientation,request.target.pose.orientation
            # q and -q represent the same rotation.
            changed = changed or abs(a.z*b.z+a.w*b.w) < 1.-1e-10
        self.target = copy.deepcopy(request.target)
        self.node.goal = [p.x, p.y, p.z]
        self.target_revision = response.target_revision = request.target_revision
        self.last_target = self.node.now_seconds()
        if changed:
            self.stable_since = None
            if self.holding and self.stop_started is None:
                self.restart_execution()
            elif self.stop_started is None:
                if self.node.client is not None and self.node.client.complete:
                    self.handoff_final_coverage(completed=True)
                else:
                    self.publish_target()
                    self.phase = 'planning'
        response.accepted, response.code = True, 'ACCEPTED'
        response.reason = 'Destination accepted; execution planning is asynchronous'
        return response

    def begin_stop(self):
        if self.stop_started is None:
            self.stop_started = time.monotonic()
            self.phase = 'stopping'
            self.node.enable(SetBool.Request(data=False), SetBool.Response(), internal=True)

    def current_pose(self):
        pose = PoseStamped()
        if self.odom is not None:
            pose.header = copy.deepcopy(self.odom.header)
            pose.pose = copy.deepcopy(self.odom.pose.pose)
        return pose

    def finish(self, success, code, reason, stopped):
        self.motion_stopped = stopped
        result = NavigateToPose.Result(success=success, code=code, reason=reason,
                                      final_pose=self.current_pose(), stop_confirmed=stopped)
        if self.handle.is_cancel_requested and code == 'CANCELED':
            self.handle.canceled()
        elif success:
            self.handle.succeed()
        else:
            self.handle.abort()
        return result

    def tick(self):
        if self.handle is None or self.finished is None or self.finished.done():
            return
        now = self.node.now_seconds()
        distance = math.inf
        if self.odom is not None:
            p = self.odom.pose.pose.position
            distance = math.dist((p.x, p.y, p.z), self.node.goal)
        request = self.handle.request
        if self.handle.is_cancel_requested:
            self.outcome = (False, 'CANCELED', 'Navigation canceled')
        elif self.holding and not -.05 <= now - self.node.pose_stamp <= .5:
            self.outcome = (False, 'EXECUTION_FAILED', 'Stale local odometry while holding')
        elif self.node.error:
            self.outcome = (False, 'EXECUTION_FAILED', self.node.error)
        elif now < self.started or now - self.started > self.timeout:
            self.outcome = (False, 'TIMEOUT', 'Navigation clock reset or task timeout')
        elif request.use_target_timeout and now - self.last_target > request.target_timeout:
            self.outcome = (False, 'TARGET_TIMEOUT', 'Navigation target stream expired')
        if self.outcome is not None:
            self.begin_stop()
        if self.stop_started is not None:
            if self.node.client is None or self.node.client.quiescent:
                self.stop_started = None
                if self.outcome is not None:
                    if self.outcome[0] and request.arrival_behavior == request.HOLD:
                        self.outcome = None
                        self.holding, self.phase = True, 'holding'
                    else:
                        self.finished.set_result(self.finish(*self.outcome, stopped=True))
                        return
                elif self.resume_after_stop:
                    self.restart_execution()
            elif time.monotonic() - self.stop_started > 8.:
                detail = self.outcome[2] if self.outcome else 'Execution handoff stop unconfirmed'
                self.finished.set_result(self.finish(False, 'STOP_UNCONFIRMED', detail, stopped=False))
                return
        elif not self.holding:
            if self.odom is not None:
                v = self.odom.twist.twist.linear
                speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
                fresh = -.05 <= now - self.node.pose_stamp <= .5
                if fresh and distance <= self.node.navigation_tolerance and speed < .3:
                    if self.stable_since is None:
                        self.stable_since = now
                else:
                    self.stable_since = None
            if self.node.client and self.node.client.complete:
                if self.stable_since is not None and now - self.stable_since >= 2.:
                    self.outcome = (True, 'SUCCEEDED', 'Execution complete and arrival held')
                    self.begin_stop()
                elif distance > self.node.navigation_tolerance:
                    # An old final trajectory completed before the updated plan arrived.
                    self.resume_after_stop = True
                    self.begin_stop()
                else:
                    self.phase = 'settling'
            elif self.node.client and self.node.client.revision:
                self.phase = 'executing'
        self.handle.publish_feedback(NavigateToPose.Feedback(
            phase=self.phase, current_pose=self.current_pose(), distance_remaining=distance,
            revision=self.node.client.revision if self.node.client else 0,
            target_revision=self.target_revision, arrived=self.holding))
