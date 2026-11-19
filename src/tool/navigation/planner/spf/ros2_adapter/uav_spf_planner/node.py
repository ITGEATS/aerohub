"""uav_spf_planner：SPF 语言导航 Planner 板端节点（迁移方案阶段 4）。

依赖闭包 = rclpy + stdlib（阶段 6 板端审计约束；禁止 numpy/cv2/pydantic/requests）。

每循环：pose/camera_info 就绪校验 → Inference Function（采图与模型调用）→
projector 真针孔反投影（D3）→ trajectory 修订（上游 rotate→forward 语义）→
每轮独立 FollowTimedPath goal → 等待段执行完毕 → 下一循环。
Navigation to_semantic_goal 无到达判定：超时 → stop → TIMEOUT，绝不报告到达成功。
旧 spf/goal 仅保留为调试入口；轮数耗尽返回 EXECUTION_FAILED。
"""
import copy
import json
import math
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.msg import TimedPath, TimedTrajectoryPoint

from .projector import project
from .trajectory import build_revision
from .vlm_client import VlmQueryClient
from .semantic_navigation import SemanticNavigationServer, validate_semantic_goal

# D13: final status carries the shared navigation result vocabulary
# (uav_planner_interfaces/action/NavigateToPose Result.code) so a future
# Mission/Tool layer consumes SPF tasks uniformly with YOPO/SUPER; the
# internal sub-reason stays in `detail`.
SHARED_CODES = {"task_timeout": "TIMEOUT", "step_limit": "EXECUTION_FAILED",
                "cancelled": "CANCELED",
                "not_ready": "NOT_READY", "goal_timeout": "TIMEOUT",
                "vlm_failures": "EXECUTION_FAILED",
                "goal_rejected": "EXECUTION_FAILED",
                "segment_failed": "EXECUTION_FAILED"}


def quat_to_yaw(q) -> float:
    """纯 stdlib：从 ROS 姿态提取 ENU 逆时针 yaw。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SpfPlannerNode(Node):
    def __init__(self):
        super().__init__("spf_planner")
        self.declare_parameter("vehicle_namespace", "drone0")
        self.declare_parameter("camera_info_topic",
                               "sensor_measurements/FrontCamera/camera_info")
        self.declare_parameter("base_velocity", 2.0)
        self.declare_parameter("base_yaw_rate", 30.0)
        self.declare_parameter("min_command_duration", 2.0)
        self.declare_parameter("adaptive_mode", False)
        self.declare_parameter("model_name", "vlm")
        self.declare_parameter("vlm_timeout_s", 30.0)
        self.declare_parameter("max_observation_age_s", 1.0)
        self.declare_parameter("max_consecutive_vlm_failures", 3)
        self.declare_parameter("vlm_retry_backoff_s", 5.0)
        self.declare_parameter("min_altitude_m", 1.2)
        self.declare_parameter("handoff_lead_s", 0.5)

        ns = self.get_parameter("vehicle_namespace").value
        self._ns = ns
        self._cfg = {k: self.get_parameter(k).value for k in (
            "base_velocity", "base_yaw_rate", "min_command_duration",
            "adaptive_mode", "vlm_timeout_s", "vlm_retry_backoff_s",
            "min_altitude_m")}

        self._lock = threading.Lock()
        self._pose_msg = PoseStamped()
        self._motion_pending = False
        self._pose = None       # (wall, x, y, z, yaw)
        self._twist_speed = None  # (wall, 3D speed m/s)
        self._intrinsics = None   # (wall, fx, fy, cx, cy, w, h, stamp_sec)
        self._state = "IDLE"
        self._job = None          # goal payload dict
        self._stop_requested = False
        self._revision = 0
        self._task_id = ""
        self._goal_handle_accepted = threading.Event()
        self._goal_result_event = threading.Event()
        self._goal_rejected = False
        self._goal_success = None
        self._vlm_attempts = 0

        # Topic/service/action names are RELATIVE to the node namespace; the node
        # is launched with __ns:=/<vehicle_namespace>, so hard-coding the ns here
        # would double-prefix (see yopo adapter for the same convention).
        self.create_subscription(PoseStamped, "self_localization/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "self_localization/twist",
                                 self._on_twist, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value,
                                 self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(String, "spf/goal", self._on_goal, 10)
        self.create_service(Trigger, "spf/cancel", self._on_cancel)
        self._status_pub = self.create_publisher(String, "spf/status", 10)

        self._action = ActionClient(self, FollowTimedPath, "follow_timed_path")
        self._stopper = self.create_client(Trigger, "follow_timed_path/_behavior/stop")
        self._vlm = VlmQueryClient(self, self.get_parameter("model_name").value)

        self._semantic = SemanticNavigationServer(self)

        self._worker = threading.Thread(target=self._work_loop, daemon=True)
        self._worker.start()
        self.create_timer(0.5, self._publish_heartbeat)
        self._publish_status("ready", reason="init")

    # ---------- 输入回调 ----------
    def _on_pose(self, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        with self._lock:
            self._pose_msg = copy.deepcopy(msg)
            self._pose = (time.monotonic(), p.x, p.y, p.z, quat_to_yaw(q))

    def _on_twist(self, msg: TwistStamped):
        v = msg.twist.linear
        with self._lock:
            self._twist_speed = (time.monotonic(), math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))

    def _on_camera_info(self, msg: CameraInfo):
        k = msg.k
        if k[0] <= 0 or k[4] <= 0:
            return
        with self._lock:
            self._intrinsics = (time.monotonic(), k[0], k[4], k[2], k[5],
                                msg.width, msg.height,
                                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _current_pose(self):
        with self._lock:
            return copy.deepcopy(self._pose_msg)

    def _start_job(self, instruction, timeout, max_cycles=None):
        with self._lock:
            self._job = {"instruction": instruction, "max_duration_s": timeout,
                         "max_cycles": max_cycles,
                         "deadline": time.monotonic() + timeout}
            self._state = "RUNNING"
            self._task_id = "spf-" + uuid.uuid4().hex
            self._revision = 0
            self._stop_requested = False
            self._motion_pending = False
            self._vlm_attempts = 0
        self._publish_status("started", instruction=instruction)

    def _on_goal(self, msg: String):
        """Legacy debugging entry; never claims semantic goal attainment."""
        try:
            payload = json.loads(msg.data)
            instruction = payload["instruction"]
            timeout = float(payload.get("max_duration_s", 120.0))
            cycles = int(payload.get("max_cycles", 8))
            validate_semantic_goal(instruction, timeout)
            if cycles <= 0:
                raise ValueError("max_cycles must be positive")
        except (ValueError, KeyError, TypeError, OverflowError):
            self._publish_status("rejected", reason="bad_goal")
            return
        with self._lock:
            busy = (self._semantic._reserved or self._state not in ("IDLE", "STOPPED"))
            if not busy:
                # Reserve before dropping the lock; the Action must not race us.
                self._state = "STARTING"
        if busy:
            self._publish_status("rejected", reason="busy")
            return
        self._start_job(instruction, timeout, max_cycles=cycles)

    def _request_stop(self):
        with self._lock:
            if self._state not in ("IDLE", "STOPPED", "FAULT"):
                self._stop_requested = True
                self._state = "STOPPING"

    def _on_cancel(self, request, response):
        self._request_stop()
        response.success = True
        response.message = "cancel requested; await terminal result for stop confirmation"
        return response

    def _interrupt_reason(self):
        with self._lock:
            if self._stop_requested:
                return "cancelled"
            if self._job and time.monotonic() >= self._job["deadline"]:
                return "task_timeout"
        return None

    def _check_interrupt(self):
        reason = self._interrupt_reason()
        if reason:
            self._do_stop(final=False, reason=reason)
        return reason is not None

    def _backoff(self):
        deadline = time.monotonic() + self._cfg["vlm_retry_backoff_s"]
        while time.monotonic() < deadline:
            if self._interrupt_reason():
                return
            time.sleep(0.05)

    # ---------- 主循环 ----------
    def _work_loop(self):
        while rclpy.ok():
            time.sleep(0.05)
            with self._lock:
                job, state = self._job, self._state
            if state not in ("RUNNING", "STOPPING"):
                continue
            if self._check_interrupt():
                continue
            if (job.get("max_cycles") is not None
                    and job.get("cycles_done", 0) >= job["max_cycles"]):
                self._do_stop(final=False, reason="step_limit")
                continue
            try:
                revision = self._run_one_cycle(job)
            except Exception as exc:
                # A malformed model response must not kill the only worker and
                # leave an accepted semantic Action running without a deadline.
                self.get_logger().error(f"SPF cycle failed: {exc}")
                self._do_stop(final=False, reason="segment_failed")
                continue
            if revision is None:
                continue  # 状态已在 _run_one_cycle 内收口
            if self._wait_segment(revision):
                with self._lock:
                    self._job["cycles_done"] = self._job.get("cycles_done", 0) + 1

    def _ready(self):
        age = self.get_parameter("max_observation_age_s").value
        with self._lock:
            ok = (self._pose and time.monotonic() - self._pose[0] < age
                  and self._intrinsics and time.monotonic() - self._intrinsics[0] < 5.0)
        return ok and self._action.server_is_ready() and self._stopper.service_is_ready()

    def _run_one_cycle(self, job):
        """执行一个 SPF 循环；返回修订号，或 None（失败已收口）。"""
        if not self._ready():
            self._do_stop(final=False, reason="not_ready")
            return None
        failures = 0
        while failures < self.get_parameter("max_consecutive_vlm_failures").value:
            if self._check_interrupt():
                return None
            self._vlm_attempts += 1
            result = self._vlm.request(job["instruction"],
                                       self._cfg["vlm_timeout_s"],
                                       interrupted=self._interrupt_reason)
            if self._check_interrupt():
                return None
            if not result.get("ok"):
                failures += 1
                self.get_logger().warn(
                    'vlm failure %d: %s' % (failures, result.get("error_code")))
                self._publish_status("vlm_failure", code=result.get("error_code"))
                # Back off on provider failures or invalid output instead of
                # immediately repeating a request.
                self._backoff()
                continue
            # Generic Function returns text; this Tool parses SPF point/depth semantics.
            if "point" not in result:
                failures += 1
                self.get_logger().warn('vlm failure %d: missing_point' % failures)
                self._publish_status("vlm_failure", code="missing_point")
                self._backoff()
                continue
            if not self._ready():
                self._do_stop(final=False, reason="not_ready")
                return None
            with self._lock:
                pose, intr = self._pose, self._intrinsics
            action = project(result["point"], result.get("depth", 4),
                             intr[1], intr[2], intr[3], intr[4],
                             intr[5], intr[6], self._cfg["adaptive_mode"])
            rev = self._send_revision(action, pose)
            if rev is None:
                self._publish_status("no_action")
                return None
            return rev
        self._do_stop(final=False, reason="vlm_failures")
        return None

    def _send_revision(self, action, pose, is_final=True):
        """Send one SPF cycle as a standalone FollowTimedPath goal.

        SPF is a discrete observe-decide-move loop: each cycle is a complete
        rotate+forward segment executed to completion before the next VLM
        observation. The behavior's rolling modify requires the replacement to
        overlap the still-running trajectory (start <= previous.end()), which a
        step-and-observe cycle cannot guarantee, so each cycle is its own goal
        with is_final set (deviation from the plan's "后续 modify" noted in
        skill/temp)."""
        if self._check_interrupt():
            return None
        if action is None:
            return None
        _, x0, y0, z0, yaw0 = pose
        plan = build_revision(action, x0, y0, z0, yaw0,
                              self._cfg["base_velocity"],
                              self._cfg["base_yaw_rate"],
                              self._cfg["min_command_duration"])
        if plan is None:
            return None
        with self._lock:
            self._revision += 1
            revision = self._revision
        path = self._make_timed_path(plan, revision, is_final)
        goal = FollowTimedPath.Goal()
        goal.path = path
        self._goal_handle_accepted.clear()
        self._goal_result_event.clear()
        self._goal_rejected = False
        self._goal_success = None
        self._motion_pending = True
        future = self._action.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        deadline = time.monotonic() + 15.0
        while not self._goal_handle_accepted.wait(0.05):
            if self._check_interrupt():
                return None
            if time.monotonic() >= deadline:
                self._do_stop(final=False, reason="goal_timeout")
                return None
        if self._goal_rejected:
            self._finish("STOPPED", "goal_rejected")
            return None
        self._plan_duration = plan["duration"]
        return revision

    def _on_goal_response(self, future):
        handle = future.result()
        if handle.accepted:
            rf = handle.get_result_async()
            rf.add_done_callback(self._on_goal_result)
            self._goal_rejected = False
        else:
            self._motion_pending = False
            self._goal_rejected = True
        self._goal_handle_accepted.set()

    def _on_goal_result(self, future):
        # GoalResult status: 4 == SUCCEEDED (rclpy action goal status).
        self._motion_pending = False
        self._goal_success = future.result().status == 4
        self._goal_result_event.set()

    def _wait_segment(self, revision):
        duration = getattr(self, "_plan_duration", 2.0)
        deadline = time.monotonic() + duration + 30.0
        while time.monotonic() < deadline:
            if self._check_interrupt():
                return
            if self._goal_result_event.wait(0.2):
                if self._goal_success:
                    return True
                self._publish_status("segment_failed", revision=revision)
                self._do_stop(final=False, reason="segment_failed")
                return
            time.sleep(0.05)
        self._publish_status("segment_timeout", revision=revision)
        self._do_stop(final=False, reason="segment_failed")

    def _do_stop(self, final: bool, reason: str = "cancelled"):
        with self._lock:
            self._state = "STOPPING"
        gate_deadline = time.monotonic() + 10.0
        stable_since = None
        stop_future = None
        stop_acked = False
        while time.monotonic() < gate_deadline:
            # Reissue until acknowledged: a goal acceptance may arrive after the
            # first stop request. Never release ownership while it is pending.
            if stop_future is not None and stop_future.done():
                try:
                    stop_acked = bool(stop_future.result().success)
                except Exception:
                    stop_acked = False
                stop_future = None
            if stop_future is None and not stop_acked:
                try:
                    stop_future = self._stopper.call_async(Trigger.Request())
                except Exception as exc:
                    self._publish_status("stop_call_failed", detail=str(exc))
            with self._lock:
                speed = self._twist_speed
            now = time.monotonic()
            fresh = speed and 0 <= now - speed[0] < 0.5
            if stop_acked and not self._motion_pending and fresh and speed[1] < 0.2:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= 1.0:
                    self._finish("STOPPED", reason)
                    return
            else:
                stable_since = None
            # If acceptance was still in flight, don't let an earlier stop ACK
            # stand in for stopping the subsequently accepted trajectory.
            if self._motion_pending and self._goal_handle_accepted.is_set():
                stop_acked = False
            time.sleep(0.05)
        self._finish("FAULT", reason)

    def _finish(self, state: str, reason: str):
        with self._lock:
            # Snapshot the task counters BEFORE dropping the job: the final
            # STOPPED status is what monitors (mission gates, Tool layer) read,
            # and _status_snapshot would otherwise report cycles_done=0.
            cycles = self._job.get("cycles_done", 0) if self._job else 0
            self._state = state
            self._job = None
            self._stop_requested = False
        terminal = dict(reason=(SHARED_CODES.get(reason, "EXECUTION_FAILED")
                                if state == "STOPPED" else "STOP_UNCONFIRMED"),
                        detail=reason, cycles_done=cycles,
                        stop_confirmed=(state == "STOPPED"))
        # Persist terminal fields for late subscribers and Action result delivery.
        self._terminal_status = terminal
        self._semantic.complete(terminal)
        self._publish_status(state, **terminal)

    def _make_timed_path(self, plan, revision, is_final) -> TimedPath:
        now = self.get_clock().now().to_msg()
        # The behavior timeline requires valid_from strictly in the future and
        # generated_at <= now (see follow_path timeline.hpp validate()). A small
        # handoff lead covers DDS transit + the behavior's processing latency so
        # the trajectory is not rejected as "late" on arrival.
        lead = float(self.get_parameter("handoff_lead_s").value)
        future = self.get_clock().now() + Duration(seconds=lead)
        path = TimedPath()
        path.interface_version = 1
        path.task_id = self._task_id
        path.vehicle_id = self._ns
        path.revision = revision
        path.frame_id = f"{self._ns}/odom"
        path.generated_at = now
        path.valid_from = future.to_msg()
        path.is_final = is_final
        path.valid_fields = 7  # VELOCITY|ACCELERATION|YAW
        path.max_speed = float(self._cfg["base_velocity"])
        for pt in plan["points"]:
            tp = TimedTrajectoryPoint()
            tp.time_from_start.sec = int(pt["t"])
            tp.time_from_start.nanosec = int(round((pt["t"] - int(pt["t"])) * 1e9))
            tp.position.x, tp.position.y, tp.position.z = pt["x"], pt["y"], pt["z"]
            tp.velocity.x, tp.velocity.y, tp.velocity.z = pt["vx"], pt["vy"], pt["vz"]
            # D12 altitude floor: the Blocks scene puts the orange ball on the
            # ground, so the VLM semantic-depth point sits below the horizon
            # and upstream's unbounded descent grinds the terrain (run z5
            # formal1: Ground_2 contact at z=0.36 during cycle 3). Clamp the
            # commanded altitude; upstream paper scenes used eye-level
            # targets and had no equivalent guard.
            if pt["t"] > 0.01 and tp.position.z < self._cfg["min_altitude_m"]:
                # The t=0 handoff point must stay at the actual position
                # (behavior continuity check), so it is never clamped.
                tp.position.z = self._cfg["min_altitude_m"]
                tp.velocity.z = 0.0
            tp.yaw = pt["yaw"]
            path.points.append(tp)
        return path

    def _status_snapshot(self, state: str) -> dict:
        # _ready() takes the lock itself; read job counters in a separate
        # short critical section to avoid re-entrant deadlock on the Lock.
        with self._lock:
            cycles = self._job.get("cycles_done", 0) if self._job else 0
            revision, task_id = self._revision, self._task_id
        return {"state": state, "task_id": task_id, "revision": revision,
                "ready": self._ready(), "cycles_done": cycles,
                "vlm_attempts": self._vlm_attempts}

    def _publish_status(self, state: str, **extra):
        payload = self._status_snapshot(state)
        payload.update(extra)
        self._status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_heartbeat(self):
        # Periodic snapshot so a late-joining monitor (mission.py) always has a
        # fresh status even between discrete events; mirrors the state machine.
        extra = getattr(self, "_terminal_status", {}) if self._state in ("STOPPED", "FAULT") else {}
        self._publish_status(self._state, **extra)


def main():
    rclpy.init()
    node = SpfPlannerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._semantic.destroy()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
