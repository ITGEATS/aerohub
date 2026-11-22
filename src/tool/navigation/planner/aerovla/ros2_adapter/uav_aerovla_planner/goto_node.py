"""Native GoTo Tool entry; preserves the original Behavior implementation.

Run this node as the sole motion Tool for a UAV. It deliberately rejects a
trajectory framework task; BehaviorExecution tasks delegate native execution to this Tool.
This harness does not claim cross-process resource arbitration or flight acceptance.
"""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from as2_msgs.action import GoToWaypoint, Land
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from . import policy
from .actions import parse_action
from .goto_steps import Convergence, action_steps
from .observation_node import AerovlaObservationNode, stamp_to_s


class AerovlaGoToNode(AerovlaObservationNode):
    def configure_execution(self):
        if self.resolved_task_id and self.resolved_view['output'] != {
                'type': 'BehaviorExecution', 'interface_version': 1}:
            raise ValueError('GoTo cannot run inside a trajectory framework session')
        settings = self.resolved_view['parameters'] if self.resolved_view else {}
        def param(name, default):
            return self.declare_parameter(name, settings.get(name, default)).value
        namespace = self.resolved_view['namespace'] if self.resolved_view else '/drone0'
        namespace = '/' + namespace.strip('/')
        self.action = ActionClient(self, GoToWaypoint,
                                   param('goto_action', namespace + '/GoToBehavior'))
        self.land_action = ActionClient(self, Land, param('land_action', namespace + '/LandBehavior'))
        self.land_timeout = float(param('land_timeout_s', 60.0))
        self.land_speed = float(param('land_speed_mps', 0.0))
        if not math.isfinite(self.land_speed) or self.land_speed < 0:
            raise ValueError('land_speed_mps must be finite and nonnegative (0 uses AS2 default)')
        self.max_age = float(param('goto_max_state_age_s', 0.5))
        self.step_timeout = float(param('goto_step_timeout_s', 30.0))
        self.stop_timeout = float(param('goto_stop_timeout_s', 10.0))
        self.tolerances = (float(param('goto_position_tolerance_m', 0.2)),
                           float(param('goto_yaw_tolerance_rad', 0.1)),
                           float(param('goto_speed_tolerance_mps', 0.15)),
                           float(param('goto_stable_s', 0.3)))
        Convergence(*self.tolerances)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.max_age, self.step_timeout, self.stop_timeout, self.land_timeout)):
            raise ValueError('GoTo timeouts and state age must be positive finite')
        self.twist = None
        self.create_subscription(TwistStamped, param('twist_topic', namespace + '/self_localization/twist'),
                                 self.on_twist, qos_profile_sensor_data)
        self.pending_steps = []
        self.active_step = None
        self.goal_handle = None
        self.native_success = False
        self.land_success = False
        self.phase = 'IDLE'
        self.not_before = float('-inf')
        self.deadline = None
        self.cancel_future = None
        self.send_pending = False
        self.stop_goal_ack = False
        self.create_timer(0.05, self.execution_tick)
        self.create_timer(0.5, self.publish_status)
        if self.resolved_task_id:
            from std_msgs.msg import String
            prefix = '/planner/tools/task_' + self.resolved_task_id.replace('-', '_')
            self.status_pub = self.create_publisher(String, prefix + '/status', 10)
            self.create_service(Trigger, prefix + '/cancel', self.on_cancel)
            self.create_service(Trigger, prefix + '/resume', self.on_resume)
            self.publish_status()

    def on_twist(self, message):
        self.twist = message

    def on_goal(self, message):
        if self.resolved_task_id:
            self.get_logger().error('Framework owns the task; external goal rejected')
            return
        if self.phase != 'IDLE' or self.state == 'RUNNING':
            self.get_logger().error('Task already active; cancel and confirm stop first')
            return
        self.land_success = False
        super().on_goal(message)

    def tick(self):
        if self.phase != 'IDLE':
            return
        super().tick()

    def same_tick_inputs(self):
        if not super().same_tick_inputs() or self.twist is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        stamps = [stamp_to_s(m.header.stamp) for m in (self.front, self.down, self.pose)]
        return (all(self.not_before < t <= now and now - t <= self.max_age for t in stamps)
                and self.state_fresh())

    def state_fresh(self):
        if self.pose is None or self.twist is None:
            return False
        if not self.pose.header.frame_id:
            return False
        if self.phase != 'IDLE' and self.pose.header.frame_id != getattr(self, 'goal_frame', self.pose.header.frame_id):
            return False
        q = self.pose.pose.orientation
        norm_squared = sum(v*v for v in (q.x, q.y, q.z, q.w))
        if not math.isfinite(norm_squared) or abs(norm_squared - 1.0) > 1e-3:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return all(0 <= now - stamp_to_s(m.header.stamp) <= self.max_age
                   for m in (self.pose, self.twist))

    def send_inference(self):
        super().send_inference()
        if self.in_flight is not None:
            self.in_flight['pose_frame'] = self.pose.header.frame_id

    def publish_status(self):
        from std_msgs.msg import String
        self.status_pub.publish(String(data=json.dumps({
            'step': self.step, 'task_id': self.task_id, 'state': self.state,
            'generation': self.generation,
            'phase': getattr(self, 'phase', 'IDLE'),
            'last_error': self.last_error, 'total_ms': self.total_ms,
            'last_gen_ids': self.last_gen_ids,
            'native_success': getattr(self, 'native_success', False),
            'land_success': getattr(self, 'land_success', False),
            'stop_confirmed': self.last_error == 'STOP_CONFIRMED',
        })))

    def on_response(self, future, ctx):
        # Expired requests cannot publish a later step, even within one task.
        if (self.in_flight is not ctx or ctx['generation'] != self.generation
                or self.state != 'RUNNING'):
            return
        self.in_flight = None
        if time.monotonic() - ctx['sent'] > self.inference_timeout:
            self.count_failure('INFERENCE_TIMEOUT')
            return
        try:
            response = future.result()
            if not response.success:
                if response.error_code != 'BUSY':
                    self.count_failure(response.error_code or 'SERVICE_FAILURE')
                return
            self.total_ms = float(response.total_ms)
            self.last_gen_ids = list(response.gen_ids)
            try:
                action = parse_action(response.text)
            except ValueError as error:
                self.count_failure('INVALID_ACTION:' + str(error))
                return
            if action is None:
                self.native_success = False
                self.land_success = False
                self.active_step = None
                self.phase = 'LAND_READY'
                self.deadline = time.monotonic() + self.land_timeout
                self.publish_status()
                return
            forward, down, yaw = action
            self.goal_frame = ctx['pose_frame']
            if not self.goal_frame:
                raise ValueError('observation pose frame is empty')
            self.pending_steps = action_steps(ctx['ned_state'], forward, down, yaw)
            self.failures = 0
            self.phase = 'READY'
            self.deadline = time.monotonic() + self.step_timeout
        except Exception as error:
            self.count_failure('GOTO_PREPARE:' + str(error))

    def start_step(self):
        if not self.action.server_is_ready():
            return
        self.active_step = self.pending_steps.pop(0)
        self.convergence = Convergence(*self.tolerances)
        self.native_success = False
        goal = GoToWaypoint.Goal()
        goal.target_pose.header.frame_id = self.goal_frame
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.point.x, goal.target_pose.point.y, goal.target_pose.point.z = self.active_step.position_enu
        goal.yaw.mode = goal.yaw.FIXED_YAW
        goal.yaw.angle = self.active_step.yaw_enu
        goal.max_speed = self.active_step.max_speed
        self.phase = 'EXECUTING'
        self.deadline = time.monotonic() + self.step_timeout
        self.send_pending = True
        self.action.send_goal_async(goal).add_done_callback(self.accepted)

    def start_land(self):
        if not self.land_action.server_is_ready():
            return
        goal = Land.Goal()
        goal.land_speed = self.land_speed
        self.phase = 'LANDING'
        self.send_pending = True
        self.land_action.send_goal_async(goal).add_done_callback(self.land_accepted)

    def land_accepted(self, future):
        self.send_pending = False
        try:
            handle = future.result()
        except Exception as error:
            self.fail('LAND_SEND:' + str(error))
            return
        if not handle.accepted:
            self.fail('LAND_REJECTED')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.land_finished)
        if self.phase in ('STOPPING', 'STOP_FAILED'):
            self.cancel_future = handle.cancel_goal_async()

    def land_finished(self, future):
        try:
            result = future.result()
        except Exception as error:
            if self.phase in ('STOPPING', 'STOP_FAILED'):
                return
            self.fail('LAND_RESULT:' + str(error))
            return
        if self.phase in ('STOPPING', 'STOP_FAILED'):
            if result.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                self.goal_handle = None
                self.stop_goal_ack = True
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.land_success:
            self.goal_handle = None
            self.fail('LAND_FAILED')
            return
        self.goal_handle = None
        self.land_success = True
        self.state = 'SUCCEEDED'
        self.phase = 'SUCCEEDED'
        self.deadline = None
        self.last_error = ''
        self.publish_status()

    def accepted(self, future):
        self.send_pending = False
        try:
            handle = future.result()
        except Exception as error:
            self.fail('GOTO_SEND:' + str(error))
            return
        if not handle.accepted:
            self.fail('GOTO_REJECTED')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.finished)
        if self.phase in ('STOPPING', 'STOP_FAILED'):
            self.cancel_future = handle.cancel_goal_async()

    def finished(self, future):
        try:
            result = future.result()
        except Exception as error:
            if self.phase not in ('STOPPING', 'STOP_FAILED') and self.last_error != 'STOP_CONFIRMED':
                self.fail('GOTO_RESULT:' + str(error))
            return
        if self.phase in ('STOPPING', 'STOP_FAILED'):
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.go_to_success:
            self.fail('GOTO_FAILED')
            return
        self.native_success = True

    def execution_tick(self):
        if self.phase in ('IDLE', 'STOP_FAILED', 'SUCCEEDED'):
            return
        now = time.monotonic()
        if self.phase == 'STOPPING':
            self.check_stop(now)
            return
        if self.deadline is not None and now > self.deadline:
            self.fail('LAND_TIMEOUT' if self.phase in ('LAND_READY', 'LANDING') else 'GOTO_STEP_TIMEOUT')
            return
        if self.phase == 'LAND_READY':
            self.start_land()
            return
        if self.phase == 'LANDING':
            return
        if self.phase == 'READY':
            if self.pending_steps:
                if self.state_fresh():
                    self.start_step()
            else:
                self.phase = 'HOLD'
                self.hold_until = now + policy.HOVER_DURATION
            return
        if self.phase == 'HOLD':
            if not self.state_fresh():
                self.fail('GOTO_STATE_STALE')
            elif self.active_step is not None and not self.measured_convergence():
                self.hold_until = now + policy.HOVER_DURATION
            elif now >= self.hold_until:
                self.phase = 'IDLE'
                self.goal_handle = None
                self.step += 1
                self.not_before = self.get_clock().now().nanoseconds * 1e-9
                self.publish_status()
            return
        if self.phase == 'EXECUTING':
            if not self.state_fresh():
                self.fail('GOTO_STATE_STALE')
                return
            converged = self.measured_convergence()
            if converged:
                self.goal_handle = None
                self.phase = 'READY'
                self.deadline = now + self.step_timeout

    def measured_convergence(self):
        pose = self.pose.pose
        yaw = math.atan2(2 * (pose.orientation.w * pose.orientation.z +
                             pose.orientation.x * pose.orientation.y),
                         1 - 2 * (pose.orientation.y**2 + pose.orientation.z**2))
        velocity = self.twist.twist.linear
        return self.convergence.update(
            self.active_step, (pose.position.x, pose.position.y, pose.position.z), yaw,
            math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2),
            stamp_to_s(self.pose.header.stamp), self.native_success)

    def fail(self, reason):
        self.last_error = reason
        self.state = 'PAUSED'
        self.begin_stop()
        self.publish_status()

    def begin_stop(self):
        self.generation += 1
        self.in_flight = None
        self.pending_steps = []
        self.phase = 'STOPPING'
        self.deadline = time.monotonic() + self.stop_timeout
        self.stop_since = None
        self.cancel_future = None
        self.stop_goal_ack = False
        if self.native_success and self.goal_handle is not None:
            # Native position SUCCESS may leave an unfinished yaw command.
            # A new position-hold goal replaces it without motion-reference IO.
            if not self.state_fresh():
                return
            pose = self.pose.pose
            goal = GoToWaypoint.Goal()
            goal.target_pose.header = self.pose.header
            goal.target_pose.point.x = pose.position.x
            goal.target_pose.point.y = pose.position.y
            goal.target_pose.point.z = pose.position.z
            goal.yaw.mode = goal.yaw.KEEP_YAW
            goal.max_speed = self.tolerances[2]
            self.action.send_goal_async(goal).add_done_callback(self.stop_hold_accepted)
        elif self.goal_handle is not None:
            self.cancel_future = self.goal_handle.cancel_goal_async()

    def stop_hold_accepted(self, future):
        try:
            handle = future.result()
            if handle.accepted:
                handle.get_result_async().add_done_callback(self.stop_hold_finished)
        except Exception:
            pass  # watchdog retains STOP_UNCONFIRMED

    def stop_hold_finished(self, future):
        try:
            result = future.result()
            self.stop_goal_ack = (result.status == GoalStatus.STATUS_SUCCEEDED
                                  and result.result.go_to_success)
        except Exception:
            self.stop_goal_ack = False

    def on_resume(self, request, response):
        if self.phase != 'IDLE' or self.last_error != 'STOP_CONFIRMED':
            response.success = False
            response.message = 'stop not confirmed'
            return response
        self.generation += 1
        self.state = 'RUNNING'
        self.last_error = ''
        self.failures = 0
        self.not_before = self.get_clock().now().nanoseconds * 1e-9
        response.success = True
        response.message = 'resumed; waiting for new observations'
        self.publish_status()
        return response

    def on_cancel(self, request, response):
        self.state = 'STOPPED'
        if self.phase not in ('STOPPING', 'STOP_FAILED') and self.last_error != 'STOP_CONFIRMED':
            self.begin_stop()
        response.success = True
        response.message = 'stop requested; status reports confirmation separately'
        self.publish_status()
        return response

    def check_stop(self, now):
        # Await send acceptance before cancel, and native cancel acknowledgement.
        ack = self.stop_goal_ack or (not self.send_pending and self.goal_handle is None)
        if self.cancel_future is not None and self.cancel_future.done():
            try:
                ack = bool(self.cancel_future.result().goals_canceling)
            except Exception:
                ack = False
        if ack and self.state_fresh():
            v = self.twist.twist.linear
            if math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) <= self.tolerances[2]:
                stamp = stamp_to_s(self.twist.header.stamp)
                if self.stop_since is None:
                    self.stop_since = stamp
                if stamp - self.stop_since >= self.tolerances[3]:
                    self.phase = 'IDLE'
                    self.goal_handle = None
                    self.last_error = 'STOP_CONFIRMED'
                    self.publish_status()
                    return
            else:
                self.stop_since = None
        else:
            self.stop_since = None
        if now > self.deadline:
            self.phase = 'STOP_FAILED'
            self.last_error = 'STOP_UNCONFIRMED'
            self.publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = AerovlaGoToNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
