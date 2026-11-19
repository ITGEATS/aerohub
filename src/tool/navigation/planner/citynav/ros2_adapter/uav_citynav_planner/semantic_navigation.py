"""Navigation Tool 的 to_semantic_goal 操作实现（CityNav，Tool 化改造 2026-11-18）。

契约与 SPF 同一共享 Action（uav_planner_interfaces/NavigateToSemanticGoal）：
description=自然语言导航指令，timeout=含推理与运动的有限正预算。结果码语义
（SUCCEEDED 保留给"已验证的目标达成"）：CityNav 的收口条件是所有 landmark
按序推进完毕（VLM 可见性确认，无位置到达检测器）——按负责人口径，本 Tool
将其映射为 success=true + code=SUCCEEDED + reason 明记
"landmark_sequence_complete_semantic_only_no_positional_verification"，位置
级验证留在任务侧离线评分（mission ground truth），该语义决策记录为偏离 D25
供负责人复核；预算耗尽/取消/故障分别 TIMEOUT/CANCELED/EXECUTION_FAILED，
取消仅在速度门控停止确认后返回 CANCELED（沿用 _do_stop 语义）。
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
        # Called while the planner task is still exclusively owned (from
        # _finish/_do_fault/_do_stop terminal publication).
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
                self._terminal = dict(code="STOP_UNCONFIRMED",
                                      detail="ROS context stopped",
                                      stop_confirmed=False)
                break
            snapshot = planner._status_snapshot(planner._state)
            feedback = NavigateToSemanticGoal.Feedback()
            feedback.phase = snapshot["state"]
            feedback.steps_completed = snapshot["step_size"]
            feedback.revision = snapshot["revision"]
            pose = planner._current_pose_msg()
            if pose is not None:
                feedback.current_pose = pose
            handle.publish_feedback(feedback)
        status = self._terminal or dict(code="EXECUTION_FAILED",
                                        detail="no terminal status",
                                        stop_confirmed=False)
        result = NavigateToSemanticGoal.Result()
        result.code = status["code"]
        result.success = status["code"] == "SUCCEEDED"
        result.reason = status["detail"]
        result.stop_confirmed = bool(status.get("stop_confirmed"))
        pose = planner._current_pose_msg()
        if pose is not None:
            result.final_pose = pose
        if result.code == "CANCELED" and handle.is_cancel_requested:
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        with planner._lock:
            self._reserved = False
        return result

    def destroy(self):
        self._server.destroy()
