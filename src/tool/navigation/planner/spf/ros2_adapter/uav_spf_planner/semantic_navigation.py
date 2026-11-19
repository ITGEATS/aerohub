"""Navigation Tool's to_semantic_goal operation, implemented by SPF.

No arrival detector is implemented: normal budget exhaustion returns TIMEOUT,
never SUCCEEDED. The planner owns motion, deadline enforcement and stop gating.
"""
import math
import threading

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from uav_planner_interfaces.action import NavigateToSemanticGoal


def validate_semantic_goal(description, timeout):
    if not isinstance(description, str) or not description.strip():
        raise ValueError("description must be non-empty")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")


class SemanticNavigationServer:
    def __init__(self, planner):
        self._planner = planner
        self._reserved = False
        self._finished = threading.Event()
        self._terminal = None
        self._server = ActionServer(
            planner, NavigateToSemanticGoal, "navigate_to_semantic_goal",
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=ReentrantCallbackGroup())

    def _goal(self, request):
        try:
            validate_semantic_goal(request.description, request.timeout)
        except (ValueError, TypeError):
            return GoalResponse.REJECT
        with self._planner._lock:
            if self._reserved or self._planner._state not in ("IDLE", "STOPPED"):
                return GoalResponse.REJECT
            self._reserved = True
            self._terminal = None
            self._finished.clear()
        return GoalResponse.ACCEPT

    def _cancel(self, handle):
        self._planner._request_stop()
        return CancelResponse.ACCEPT

    def complete(self, status):
        # Called while the planner task is still exclusively owned.
        if self._reserved:
            self._terminal = status
            self._finished.set()

    def _execute(self, handle):
        planner = self._planner
        planner._start_job(handle.request.description, handle.request.timeout)
        if handle.is_cancel_requested:
            planner._request_stop()
        while not self._finished.wait(0.1):
            if not planner.context.ok():
                self._terminal = dict(reason="STOP_UNCONFIRMED",
                                      detail="ROS context stopped", stop_confirmed=False)
                break
            snapshot = planner._status_snapshot(planner._state)
            feedback = NavigateToSemanticGoal.Feedback()
            feedback.phase = snapshot["state"]
            feedback.steps_completed = snapshot["cycles_done"]
            feedback.revision = snapshot["revision"]
            feedback.current_pose = planner._current_pose()
            handle.publish_feedback(feedback)
        status = self._terminal
        result = NavigateToSemanticGoal.Result()
        result.success = False
        result.code = status["reason"]
        result.reason = status["detail"]
        result.stop_confirmed = status["stop_confirmed"]
        result.final_pose = planner._current_pose()
        if result.code == "CANCELED" and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        with planner._lock:
            self._reserved = False
        return result

    def destroy(self):
        self._server.destroy()
