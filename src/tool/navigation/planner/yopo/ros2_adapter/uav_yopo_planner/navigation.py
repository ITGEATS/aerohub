"""Shared navigation Action over YOPO's existing rolling/stop lifecycle."""
import copy
import math
import threading

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from uav_planner_interfaces.action import NavigateToPose
from uav_planner_interfaces.srv import UpdateNavigationTarget


class YopoNavigation:
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.pending = False
        self.reserved = False
        self.handle = self.finished = None
        self.started = False
        self.target_revision = 0
        self.holding = False
        self.last_target = 0.
        self.server = ActionServer(
            node, NavigateToPose, 'navigate_to_pose', execute_callback=self.execute,
            goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.update_service = node.create_service(
            UpdateNavigationTarget, 'navigate_to_pose/update_target', self.update_target)
        self.timer = node.create_timer(.02, self.tick)

    def accept(self, request):
        with self.lock:
            if self.reserved or self.node.state not in ('IDLE', 'STOPPED') or self.node.queued_goal is not None:
                return GoalResponse.REJECT
            self.reserved = True
            return GoalResponse.ACCEPT

    def validate(self, request):
        n = self.node
        p = request.target.pose.position
        if request.target.header.frame_id != n.frame or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return 'INVALID_GOAL', 'Finite position in ' + n.frame + ' required'
        if request.orientation_mode != request.UNCONSTRAINED:
            return 'UNSUPPORTED', 'YOPO controls yaw along its reference; terminal yaw constraints are unsupported'
        if request.arrival_behavior not in (request.COMPLETE, request.HOLD):
            return 'INVALID_GOAL', 'Unknown arrival behavior'
        if request.arrival_behavior == request.HOLD and not request.use_target_timeout:
            return 'INVALID_GOAL', 'HOLD requires a target stream timeout'
        for name in ('max_speed', 'position_tolerance', 'timeout', 'target_timeout'):
            if getattr(request, 'use_' + name):
                value = getattr(request, name)
                if not math.isfinite(value) or value <= 0:
                    return 'INVALID_GOAL', name + ' must be finite and positive'
        if request.use_max_speed and not math.isclose(request.max_speed, n.max_speed):
            return 'UNSUPPORTED', 'Per-task speed changes unsupported; use deployed speed'
        if request.use_position_tolerance and request.position_tolerance < 5.:
            return 'UNSUPPORTED', 'YOPO retains its original distance < 5 m arrival condition'
        if not n.observations_ready(n.now_s()) or not n.ready():
            return 'NOT_READY', 'YOPO observations, geometry or execution/inference services unavailable'
        return None

    async def execute(self, handle):
        self.handle, self.finished = handle, Future()
        self.started = False
        self.deadline_reason = None
        self.terminal_reason = None
        # Start in the node's timer callback group, serialized with its original
        # planning timer. Action callbacks may run on another executor thread.
        self.pending = True
        try:
            return await self.finished
        finally:
            self.handle = self.finished = None
            self.started = self.pending = False
            with self.lock:
                self.reserved = False

    def finish(self, success, code, reason):
        pose = copy.deepcopy(self.node.pose) if self.node.pose is not None else PoseStamped()
        result = NavigateToPose.Result(
            success=success, code=code, reason=reason, final_pose=pose,
            stop_confirmed=self.node.state in ('IDLE', 'STOPPED'))
        if code == 'CANCELED' and self.handle.is_cancel_requested:
            self.handle.canceled()
        elif success:
            self.handle.succeed()
        else:
            self.handle.abort()
        return result

    def update_target(self, request, response):
        response.target_revision = self.target_revision
        n, h = self.node, self.handle
        error = None
        if h is None or not self.started or self.finished is None or self.finished.done():
            error = ('NO_ACTIVE_TASK', 'No active navigation task')
        elif request.goal_id != h.goal_id:
            error = ('NOT_OWNER', 'Goal UUID does not own this task')
        elif h.is_cancel_requested or n.state not in ('RUNNING', 'STOPPED') or (n.state == 'STOPPED' and not self.holding):
            error = ('STOPPING', 'Navigation is stopping')
        elif n.now_s() < self.start_time or n.now_s() - self.start_time > self.timeout:
            error = ('TIMEOUT', 'Navigation task deadline expired')
        elif h.request.use_target_timeout and n.now_s() - self.last_target > h.request.target_timeout:
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
        changed = list(n.goal) != [p.x, p.y, p.z]
        self.target_revision = response.target_revision = request.target_revision
        self.last_target = n.now_s()
        if changed:
            if self.holding:
                n.begin(request.target)
                self.holding = False
            else:
                # Preserve the current execution and accepted reference timeline.
                n.goal = n.goal.copy()
                n.goal[:] = [p.x, p.y, p.z]
        n.target_revision = self.target_revision
        response.accepted, response.code = True, 'ACCEPTED'
        response.reason = 'Destination accepted; execution planning is asynchronous'
        return response

    def tick(self):
        if self.pending:
            self.pending = False
            error = self.validate(self.handle.request)
            if error:
                self.finished.set_result(self.finish(False, *error))
                return
            if self.handle.is_cancel_requested:
                self.finished.set_result(self.finish(False, 'CANCELED', 'Canceled before execution'))
                return
            self.start_time = self.last_target = self.node.now_s()
            self.timeout = self.handle.request.timeout if self.handle.request.use_timeout else 300.
            self.node.begin(self.handle.request.target)
            self.target_revision = self.node.target_revision = 1
            self.holding = False
            self.started = True
        if not self.started or self.finished is None or self.finished.done():
            return
        n, h = self.node, self.handle
        now = n.now_s()
        deadline_code = None
        if h.is_cancel_requested:
            deadline_code = 'CANCELED'
        elif now < self.start_time or now - self.start_time > self.timeout:
            deadline_code = 'NAVIGATION_TIMEOUT'
            self.deadline_reason = 'Navigation clock reset or task timeout'
        elif h.request.use_target_timeout and now - self.last_target > h.request.target_timeout:
            deadline_code = 'TARGET_TIMEOUT'
        elif self.holding and not n.inputs_fresh(now):
            deadline_code = 'STALE_OR_INVALID_INPUT'
        if deadline_code:
            self.holding = False
            # An arrival stop may already be draining; keep the explicit terminal
            # reason so cancellation/timeouts cannot turn into arrival success.
            self.terminal_reason = deadline_code
            if n.state == 'RUNNING':
                n.stop(deadline_code)
        if n.state in ('STOPPED', 'FAULT'):
            reason = getattr(self, 'terminal_reason', None) or n.reason
            if n.state == 'FAULT':
                outcome = False, 'STOP_UNCONFIRMED', n.reason
            elif reason == 'ARRIVED_5M' and h.request.arrival_behavior == h.request.HOLD:
                self.holding = True
                outcome = None
            elif reason == 'ARRIVED_5M':
                outcome = True, 'SUCCEEDED', reason
            elif reason == 'CANCELED':
                outcome = False, 'CANCELED', reason
            elif reason == 'NAVIGATION_TIMEOUT':
                outcome = False, 'TIMEOUT', self.deadline_reason
            elif reason == 'TARGET_TIMEOUT':
                outcome = False, 'TARGET_TIMEOUT', 'Navigation target stream expired'
            else:
                outcome = False, 'EXECUTION_FAILED', reason
            if outcome is not None:
                self.finished.set_result(self.finish(*outcome))
                return
        pose = copy.deepcopy(n.pose) if n.pose is not None else PoseStamped()
        p = pose.pose.position
        phase = ('holding' if self.holding else 'stopping' if n.state == 'STOPPING'
                 else 'executing' if n.timeline.revision else 'planning')
        h.publish_feedback(NavigateToPose.Feedback(
            phase=phase, current_pose=pose,
            distance_remaining=math.dist((p.x, p.y, p.z), n.goal),
            revision=n.timeline.revision, target_revision=self.target_revision,
            arrived=self.holding))
