"""Following Tool backed by Elastic and the shared observation-only TrackObject."""
import json
import math
from threading import Lock
from time import monotonic, sleep

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import SetBool
from uav_motion_resource.lease import MotionLease
from uav_planner_interfaces.action import FollowObject
from uav_target_tracking_interfaces.action import TrackObject

from .elastic_tracking_bridge import TargetStateBridge


class ElasticFollowing(TargetStateBridge):
    def __init__(self):
        super().__init__(node_name='elastic_following')
        callbacks = ReentrantCallbackGroup()
        self.observe = ActionClient(
            self, TrackObject, 'tools/target_tracking/track', callback_group=callbacks)
        self.control = self.create_client(SetBool, 'elastic_adapter/enable', callback_group=callbacks)
        self.create_subscription(String, 'elastic_adapter/state', self.update_state, 10,
                                 callback_group=callbacks)
        self.server = ActionServer(
            self, FollowObject, 'tools/following/follow', execute_callback=self.execute,
            goal_callback=self.admit, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=callbacks)
        self.slot = Lock()
        self.motion = MotionLease(self.get_namespace())
        self.state = ({}, 0, float('-inf'))
        self.observation_token = None
        self.preparation_timeout = 5.0
        self.stop_timeout = 5.0

    def update_state(self, message):
        self.state = (json.loads(message.data), self.state[1] + 1, monotonic())

    def admit(self, request):
        if (request.selection_mode != FollowObject.Goal.QUERY
                or not request.query.strip() or not math.isfinite(request.timeout)
                or request.timeout <= 0 or not self.observe.server_is_ready()
                or not self.control.service_is_ready() or not self.slot.acquire(blocking=False)):
            return GoalResponse.REJECT
        state, _, received = self.state
        if (not state.get('stop_confirmed') or monotonic() - received > 1.0
                or not self.motion.acquire('following.elastic')):
            self.slot.release()
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def observation_feedback(self, token, message):
        if token is not self.observation_token:
            return
        self.forward(message.feedback.state)

    def wait_for(self, condition, deadline, goal=None):
        while rclpy.ok() and monotonic() < deadline:
            if goal is not None and goal.is_cancel_requested:
                return False
            if condition():
                return True
            sleep(.02)
        return False

    def release_observation(self, future):
        self.observation_token = None

        def cancel_when_accepted(submission):
            if submission.exception() is None:
                handle = submission.result()
                if handle.accepted:
                    handle.cancel_goal_async()

        if future is not None:
            future.add_done_callback(cancel_when_accepted)

    def execute(self, goal):
        observation = None
        motion_requested = False
        stopped = True
        code, reason = 'FAILED', 'Following ended unexpectedly'
        try:
            deadline = monotonic() + goal.request.timeout
            preparation = min(deadline, monotonic() + self.preparation_timeout)
            token = object()
            self.session = None
            self.last_stamp_ns = 0
            self.observation_token = token
            observation = self.observe.send_goal_async(
                TrackObject.Goal(query=goal.request.query.strip(), follow=False,
                                 timeout=goal.request.timeout + self.stop_timeout),
                feedback_callback=lambda message: self.observation_feedback(token, message))
            if not self.wait_for(observation.done, preparation, goal) or not observation.result().accepted:
                code, reason = 'UNAVAILABLE', 'Tracking goal was not accepted within the preparation window'
            elif not self.wait_for(self.ready_to_follow, preparation, goal):
                code, reason = 'UNAVAILABLE', 'Tracking and vehicle inputs did not become ready'
            else:
                observation_result = observation.result().get_result_async()
                motion_requested = True
                enable = self.control.call_async(SetBool.Request(data=True))
                if not self.wait_for(enable.done, preparation, goal) or not enable.result().success:
                    code, reason = 'UNAVAILABLE', 'Elastic did not accept the motion task'
                else:
                    code, reason = self.follow(goal, observation_result, deadline)
            if goal.is_cancel_requested:
                code, reason = 'CANCELED', 'Canceled by caller'
        except Exception as error:
            reason = str(error)
        finally:
            try:
                if motion_requested:
                    stopped = self.stop_motion()
            except Exception as error:
                stopped = False
                reason = str(error)
            finally:
                if stopped:
                    self.release_observation(observation)
                else:
                    # Preserve the owned observation until its finite lease ends.
                    self.observation_token = None
                self.motion.release(stopped)
                self.slot.release()

        if not stopped:
            code, reason = 'FAILED', f'{reason}. Motion stop was not confirmed'
        if code == 'CANCELED':
            goal.canceled()
        elif code == 'TIMEOUT':
            goal.succeed()
        else:
            goal.abort()
        return FollowObject.Result(code=code, reason=reason, stop_confirmed=stopped)

    def ready_to_follow(self):
        state, _, received = self.state
        return self.last_stamp_ns > 0 and state.get('input_ready') and monotonic() - received <= 1.0

    def follow(self, goal, observation_result, deadline):
        while rclpy.ok() and monotonic() < deadline:
            if goal.is_cancel_requested:
                return 'CANCELED', 'Canceled by caller'
            if observation_result.done():
                return 'FAILED', 'Tracking observation session ended'
            state, _, received = self.state
            if monotonic() - received > 1.0:
                return 'FAILED', 'Elastic status became unavailable'
            if state.get('phase') == 'failed':
                return 'FAILED', state.get('error') or 'Elastic reported a failure'
            goal.publish_feedback(FollowObject.Feedback(
                phase=state.get('phase', ''), target_ready=bool(state.get('input_ready')),
                reference_ready=bool(state.get('reference_ready'))))
            sleep(.1)
        if not rclpy.ok():
            return 'FAILED', 'Following node shut down'
        return 'TIMEOUT', 'Following time elapsed'

    def stop_motion(self):
        deadline = monotonic() + self.stop_timeout
        if not self.wait_for(self.control.service_is_ready, deadline):
            return False
        stop = self.control.call_async(SetBool.Request(data=False))
        if not self.wait_for(stop.done, deadline) or not stop.result().success:
            return False
        sequence = self.state[1]

        def stopped():
            state, observed, _ = self.state
            return observed > sequence and not state.get('enabled') and state.get('stop_confirmed')

        return self.wait_for(stopped, deadline)


def main():
    rclpy.init()
    node = ElasticFollowing()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
