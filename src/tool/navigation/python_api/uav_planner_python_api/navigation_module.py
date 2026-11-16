"""AS2 Module for the planner-independent NavigateToPose Action.

The DroneInterface executor must be spinning. Blocking methods are intended for
application threads, not callbacks of that executor.
"""
import copy
import time
import threading

from action_msgs.msg import GoalStatus
from as2_python_api.modules.module_base import ModuleBase
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from uav_planner_interfaces.action import NavigateToPose
from uav_planner_interfaces.srv import UpdateNavigationTarget


class NavigationModule(ModuleBase):
    __alias__ = 'navigation'

    def __init__(self, drone):
        super().__init__(drone, self.__alias__)
        self._drone = drone
        self._client = ActionClient(drone, NavigateToPose, 'navigate_to_pose')
        self._updates = drone.create_client(UpdateNavigationTarget, 'navigate_to_pose/update_target')
        self._submission_lock = threading.RLock()
        self._issued_revision = self._target_revision = 0
        self._canceling = False
        self._goal = None
        self._handle = None
        self._result_future = None
        self._feedback = None

    @property
    def ready(self):
        return self._client.server_is_ready()

    @property
    def feedback(self):
        return self._feedback

    @property
    def done(self):
        return self._result_future is not None and self._result_future.done()

    @property
    def result(self):
        return self._result_future.result().result if self.done else None

    @property
    def result_status(self):
        return self._result_future.result().status if self.done else None

    def _wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if not self._drone.context.ok():
                raise RuntimeError('ROS context stopped')
            if time.monotonic() >= deadline:
                raise TimeoutError('Navigation response timeout')
            time.sleep(.01)
        return future.result()

    def __call__(self, target, orientation_mode=NavigateToPose.Goal.UNCONSTRAINED,
                 max_speed=None, position_tolerance=None, timeout=None, wait=True,
                 response_timeout=10.0, arrival_behavior=None, target_timeout=None):
        """Start navigation or update this Module's active task destination.

        Task policies are immutable until completion/cancel. Omitted policies on
        updates inherit the active task. HOLD sessions require wait=False; use
        wait_until_arrived() to wait for arrival without ending the session.
        """
        with self._submission_lock:
            active = self._result_future is not None and not self.done
            if active:
                self._check_policies(max_speed, position_tolerance, timeout,
                                     arrival_behavior, target_timeout)
                if self._canceling:
                    raise RuntimeError('Navigation is stopping')
                if wait and self._goal.arrival_behavior == NavigateToPose.Goal.HOLD:
                    raise ValueError('HOLD navigation requires wait=False; use wait_until_arrived()')
                self._update_target(target, orientation_mode, response_timeout)
            else:
                if self.done and not self.result.stop_confirmed:
                    raise RuntimeError('Previous navigation stop is unconfirmed')
                behavior = 'complete' if arrival_behavior is None else arrival_behavior
                if behavior not in ('complete', 'hold'):
                    raise ValueError('arrival_behavior must be complete or hold')
                if behavior == 'hold' and wait:
                    raise ValueError('HOLD navigation requires wait=False; use wait_until_arrived()')
                if not self._client.wait_for_server(timeout_sec=response_timeout):
                    raise TimeoutError('navigate_to_pose server unavailable')
                goal = NavigateToPose.Goal()
                goal.target = copy.deepcopy(target)
                goal.orientation_mode = orientation_mode
                goal.arrival_behavior = (goal.HOLD if behavior == 'hold' else goal.COMPLETE)
                # A following Mission must renew its target even when stationary.
                if target_timeout is None and behavior == 'hold':
                    target_timeout = 5.0
                for field, value in [('max_speed', max_speed),
                                     ('position_tolerance', position_tolerance),
                                     ('timeout', timeout), ('target_timeout', target_timeout)]:
                    setattr(goal, 'use_' + field, value is not None)
                    if value is not None:
                        setattr(goal, field, float(value))
                self._feedback = None
                future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
                try:
                    handle = self._wait(future, response_timeout)
                except TimeoutError:
                    future.add_done_callback(self._cancel_late_goal)
                    raise
                if not handle.accepted:
                    raise RuntimeError('Navigation goal rejected (busy or invalid)')
                self._handle, self._goal = handle, goal
                self._issued_revision = self._target_revision = 1
                self._canceling = False
                self._result_future = handle.get_result_async()
            task_timeout = self._goal.timeout if self._goal.use_timeout else 300.
        if wait:
            return self.wait_to_result(task_timeout + 15.)
        return True  # Destination accepted, not necessarily planned/executed yet.

    def _check_policies(self, max_speed, position_tolerance, timeout,
                        arrival_behavior, target_timeout):
        for name, value in [('max_speed', max_speed), ('position_tolerance', position_tolerance),
                            ('timeout', timeout), ('target_timeout', target_timeout)]:
            if value is not None and (not getattr(self._goal, 'use_' + name) or
                                      float(value) != getattr(self._goal, name)):
                raise ValueError(name + ' cannot change during a navigation task')
        if arrival_behavior is not None:
            current = 'hold' if self._goal.arrival_behavior == self._goal.HOLD else 'complete'
            if arrival_behavior != current:
                raise ValueError('arrival_behavior cannot change during a navigation task')

    def _update_target(self, target, orientation_mode, response_timeout):
        if orientation_mode != self._goal.orientation_mode:
            raise ValueError('orientation_mode cannot change during a navigation task')
        if not self._updates.wait_for_service(timeout_sec=response_timeout):
            raise TimeoutError('navigate_to_pose/update_target unavailable')
        self._issued_revision += 1
        request = UpdateNavigationTarget.Request(
            goal_id=self._handle.goal_id, target_revision=self._issued_revision,
            target=copy.deepcopy(target), orientation_mode=orientation_mode)
        try:
            response = self._wait(self._updates.call_async(request), response_timeout)
        except TimeoutError:
            # Acceptance is uncertain; stop the owned task, including any late update.
            self.cancel_async()
            raise
        if not response.accepted:
            raise RuntimeError('Navigation update rejected: ' + response.code + ': ' + response.reason)
        if response.target_revision != request.target_revision:
            self.cancel_async()
            raise RuntimeError('Navigation update response revision mismatch')
        self._target_revision = response.target_revision
        self._goal.target = copy.deepcopy(target)

    def wait_until_arrived(self, timeout=315.):
        """Wait for the current destination, leaving a HOLD session active."""
        if self._result_future is None:
            raise RuntimeError('No navigation task')
        revision = self._target_revision
        deadline = time.monotonic() + timeout
        while True:
            feedback = self._feedback
            if feedback is not None and feedback.target_revision == revision and feedback.arrived:
                return True
            if self.done:
                return bool(self.result.success and self._target_revision == revision)
            if self._target_revision != revision:
                raise RuntimeError('Destination changed while waiting for arrival')
            if not self._drone.context.ok():
                raise RuntimeError('ROS context stopped')
            if time.monotonic() >= deadline:
                self.cancel()
                raise TimeoutError('Navigation arrival timeout')
            time.sleep(.01)

    @staticmethod
    def _cancel_late_goal(future):
        handle = future.result()
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()

    def _on_feedback(self, message):
        # Late feedback from a completed session must not satisfy arrival waits
        # in a new task whose destination revision also starts at one.
        if self._handle is not None and message.goal_id == self._handle.goal_id:
            self._feedback = message.feedback

    def to_point(self, x, y, z, frame_id=None, **kwargs):
        target = PoseStamped()
        target.header.frame_id = frame_id or self._drone.drone_id + '/odom'
        target.header.stamp = self._drone.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, (x, y, z))
        return self(target, orientation_mode=NavigateToPose.Goal.UNCONSTRAINED, **kwargs)

    def to_pose(self, target, orientation_mode=NavigateToPose.Goal.TARGET_ORIENTATION, **kwargs):
        return self(target, orientation_mode=orientation_mode, **kwargs)

    def wait_to_result(self, timeout=315.):
        if self._result_future is None:
            raise RuntimeError('No navigation task')
        try:
            wrapped = self._wait(self._result_future, timeout)
        except TimeoutError:
            self.cancel()
            raise
        return wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success

    def cancel_async(self):
        """Request cancellation; the returned future is only its acknowledgement."""
        with self._submission_lock:
            if self._handle is None or self.done:
                return None
            self._canceling = True
            return self._handle.cancel_goal_async()

    def cancel(self, timeout=10.):
        if self._handle is None:
            return True
        if not self.done:
            self._wait(self.cancel_async(), timeout)
            self._wait(self._result_future, timeout)
        if not self.result.stop_confirmed:
            raise RuntimeError('Navigation stop unconfirmed: ' + self.result.reason)
        return True

    def destroy(self):
        self._client.destroy()
        self._drone.destroy_client(self._updates)
