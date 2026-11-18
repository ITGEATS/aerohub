"""AS2 Module for the planner-independent NavigateToPose Action.

The DroneInterface executor must be spinning. Blocking methods are intended for
application threads, not callbacks of that executor.
"""
import copy
import math
import time

from action_msgs.msg import GoalStatus
from as2_python_api.modules.module_base import ModuleBase
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from uav_planner_interfaces.action import NavigateToPose, NavigateToSemanticGoal


class NavigationModule(ModuleBase):
    __alias__ = 'navigation'

    def __init__(self, drone):
        super().__init__(drone, self.__alias__)
        self._drone = drone
        self._client = ActionClient(drone, NavigateToPose, 'navigate_to_pose')
        self._semantic_client = ActionClient(
            drone, NavigateToSemanticGoal, 'navigate_to_semantic_goal')
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
                 response_timeout=10.0):
        goal = NavigateToPose.Goal()
        goal.target = copy.deepcopy(target)
        goal.orientation_mode = orientation_mode
        for field, value in [('max_speed', max_speed),
                             ('position_tolerance', position_tolerance), ('timeout', timeout)]:
            setattr(goal, 'use_' + field, value is not None)
            if value is not None:
                setattr(goal, field, float(value))
        return self._submit(self._client, goal, timeout, wait, response_timeout)

    @property
    def semantic_ready(self):
        """Whether this deployment exposes the semantic operation."""
        return self._semantic_client.server_is_ready()

    def to_semantic_goal(self, description, timeout, wait=True, response_timeout=10.0):
        """Navigate using a spatial description and a required timeout.

        SPF has no arrival detector: it returns TIMEOUT (False), CANCELED or an
        error, never a successful arrival. Inspect result.code and stop_confirmed.
        """
        if not isinstance(description, str) or not description.strip():
            raise ValueError('description must be non-empty')
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        goal = NavigateToSemanticGoal.Goal()
        goal.description = description
        goal.timeout = timeout
        return self._submit(self._semantic_client, goal, timeout, wait, response_timeout)

    def _submit(self, client, goal, timeout, wait, response_timeout):
        if self._result_future is not None and not self.done:
            raise RuntimeError('Navigation is already running')
        if not client.wait_for_server(timeout_sec=response_timeout):
            raise TimeoutError('Requested navigation operation unavailable')
        self._feedback = None
        future = client.send_goal_async(goal, feedback_callback=self._on_feedback)
        try:
            handle = self._wait(future, response_timeout)
        except TimeoutError:
            # A delayed acceptance must not leave an unowned flight task.
            future.add_done_callback(self._cancel_late_goal)
            raise
        if not handle.accepted:
            raise RuntimeError('Navigation goal rejected (busy or invalid)')
        self._handle = handle
        self._result_future = handle.get_result_async()
        if wait:
            return self.wait_to_result((timeout if timeout is not None else 300.) + 15.)
        return True  # Accepted; completion is exposed separately.

    @staticmethod
    def _cancel_late_goal(future):
        handle = future.result()
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()

    def _on_feedback(self, message):
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
        if self._handle is None or self.done:
            return None
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
        self._semantic_client.destroy()
