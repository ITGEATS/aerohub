#!/usr/bin/env python3
"""Expose the common Following action for the deployed YOPO-Tracker runtime."""
import json
import math
from threading import Lock
from time import monotonic, sleep

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_motion_resource.lease import MotionLease
from uav_planner_interfaces.action import FollowObject


class YopoFollowing(Node):
    def __init__(self):
        super().__init__('yopo_following', namespace='drone0')
        group = ReentrantCallbackGroup()
        backend = self.declare_parameter('backend', '/yopo_tracker_live').value.rstrip('/')
        self.start = self.create_client(Trigger, backend + '/start', callback_group=group)
        self.stop = self.create_client(Trigger, backend + '/stop', callback_group=group)
        self.events = {}
        self.slot = Lock()
        self.motion = MotionLease(self.get_namespace())
        self.unresolved = False
        self.create_subscription(String, backend + '/status', self.on_event, 20)
        self.action = ActionServer(
            self, FollowObject, 'tools/following/follow', execute_callback=self.execute,
            goal_callback=self.admit, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)

    def admit(self, goal):
        if (goal.selection_mode != FollowObject.Goal.AUTO or goal.query
                or not math.isfinite(goal.timeout) or goal.timeout <= 0
                or not self.start.service_is_ready() or not self.stop.service_is_ready()
                or not self.slot.acquire(blocking=False)):
            return GoalResponse.REJECT
        if self.unresolved or not self.motion.acquire('following.yopo-tracker'):
            self.slot.release()
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_event(self, message):
        try:
            data = json.loads(message.data)
        except ValueError:
            return
        task = data.get('task_id')
        if task:
            self.events.setdefault(task, {})[data['event']] = data

    @staticmethod
    def wait_until(condition, deadline):
        while rclpy.ok() and monotonic() < deadline:
            if condition():
                return True
            sleep(.02)
        return False

    def execute(self, handle):
        deadline = monotonic() + handle.request.timeout
        code, reason, stopped = 'FAILED', 'YOPO-Tracker action did not complete', True
        try:
            self.events.clear()
            if handle.is_cancel_requested:
                code, reason = 'CANCELED', 'Canceled by caller'
            else:
                stopped = False
                started = self.start.call_async(Trigger.Request())
                if not self.wait_until(started.done, monotonic() + 5.0):
                    reason = 'YOPO-Tracker start was not confirmed'
                    started.add_done_callback(self.stop_late_start)
                elif not started.result().success:
                    stopped = True
                    code, reason = 'UNAVAILABLE', started.result().message
                else:
                    code, reason, stopped = self.run_task(handle, started.result().message, deadline)
        except Exception as error:
            reason = str(error)
        finally:
            self.unresolved = not stopped
            self.motion.release(stopped)
            self.slot.release()

        if not stopped:
            code, reason = 'FAILED', f'{reason}. Motion stop was not confirmed'
        if code == 'CANCELED':
            handle.canceled()
        elif code == 'TIMEOUT':
            handle.succeed()
        else:
            handle.abort()
        return FollowObject.Result(code=code, reason=reason, stop_confirmed=stopped)

    def stop_late_start(self, future):
        if future.exception() is None and future.result().success:
            self.stop.call_async(Trigger.Request())

    def run_task(self, handle, task, deadline):
        events = self.events.setdefault(task, {})
        while rclpy.ok() and monotonic() < deadline and not handle.is_cancel_requested:
            if 'finished' in events or 'brake_latched' in events:
                break
            active = 'active' in events
            inference = events.get('inference', {})
            handle.publish_feedback(FollowObject.Feedback(
                phase='following' if active else 'acquiring',
                target_ready=inference.get('target_enu') is not None, reference_ready=active))
            sleep(.1)

        if 'finished' in events or 'brake_latched' in events:
            code, reason = 'FAILED', events.get('brake_latched', events.get('finished', {})).get(
                'reason', 'YOPO-Tracker exited following')
        elif handle.is_cancel_requested:
            code, reason = 'CANCELED', 'Canceled by caller'
        else:
            code, reason = 'TIMEOUT', 'Following time elapsed'

        stop_deadline = monotonic() + 15.0
        if 'finished' not in events and 'brake_latched' not in events:
            stop = self.stop.call_async(Trigger.Request())
            if not self.wait_until(stop.done, stop_deadline) or not stop.result().success:
                return 'FAILED', 'YOPO-Tracker did not acknowledge the stop request', False

        self.wait_until(lambda: 'finished' in events, stop_deadline)
        finished = events.get('finished', {})
        never_started = (finished.get('revision') == 0
                         and 'accepted' not in events and 'active' not in events)
        return code, reason, bool(finished.get('success') or never_started)


def main():
    rclpy.init()
    node = YopoFollowing()
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


if __name__ == '__main__':
    main()
