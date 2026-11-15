"""Shared navigation Action over YOPO's existing rolling/stop lifecycle."""
import copy
import math
import threading

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from uav_planner_interfaces.action import NavigateToPose


class YopoNavigation:
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.pending = False
        self.reserved = False
        self.handle = self.finished = None
        self.started = False
        self.server = ActionServer(
            node, NavigateToPose, 'navigate_to_pose', execute_callback=self.execute,
            goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
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
        for name in ('max_speed', 'position_tolerance', 'timeout'):
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
            self.start_time = self.node.now_s()
            self.timeout = self.handle.request.timeout if self.handle.request.use_timeout else 300.
            self.node.begin(self.handle.request.target)
            self.started = True
        if not self.started or self.finished is None or self.finished.done():
            return
        n, h = self.node, self.handle
        now = n.now_s()
        if n.state == 'RUNNING':
            if h.is_cancel_requested:
                n.stop('CANCELED')
            elif now < self.start_time or now - self.start_time > self.timeout:
                self.deadline_reason = 'Navigation clock reset or task timeout'
                n.stop('NAVIGATION_TIMEOUT')
        if n.state in ('STOPPED', 'FAULT'):
            reason = n.reason
            if n.state == 'FAULT':
                outcome = False, 'STOP_UNCONFIRMED', reason
            elif reason == 'ARRIVED_5M':
                outcome = True, 'SUCCEEDED', reason
            elif reason == 'CANCELED':
                outcome = False, 'CANCELED', reason
            elif reason == 'NAVIGATION_TIMEOUT':
                outcome = False, 'TIMEOUT', self.deadline_reason
            else:
                outcome = False, 'EXECUTION_FAILED', reason
            self.finished.set_result(self.finish(*outcome))
            return
        pose = copy.deepcopy(n.pose) if n.pose is not None else PoseStamped()
        p, target = pose.pose.position, h.request.target.pose.position
        phase = 'stopping' if n.state == 'STOPPING' else ('executing' if n.timeline.revision else 'planning')
        h.publish_feedback(NavigateToPose.Feedback(
            phase=phase, current_pose=pose,
            distance_remaining=math.dist((p.x, p.y, p.z), (target.x, target.y, target.z)),
            revision=n.timeline.revision))
