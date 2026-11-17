"""uav_citynav_planner：CityNavAgent 语言导航 Planner 板端节点（迁移方案阶段 5）。

依赖闭包 = rclpy + numpy/scipy + stdlib（onboard-computer v0.1.1 容器 py3.10
实测无 networkx，D24 包内最小图替代；禁
torch/cv2/sklearn——grounding/CLIP 走 Inference HTTP 服务，统计滤波 cKDTree）。

状态机仿 spf node：IDLE/RUNNING/STOPPING/STOPPED/FAULT；generation 在 goal
启动/cancel 时递增并作废异步回调（仿 yopo），revision 为 FollowTimedPath 修订
计数。每探索循环：五目批次取齐（view_freshness_s=2.0 s 整批作废，契约 §4）→
exploit 命中检查（find_closest_node 20 m）→ HSPM（landmark 抽取一次 +
viewpoint 查询 + grounding 分割 + ENU 反投影 + alpha=0.6 收缩 waypoint）→
calculate_movement_steps 分解 → FollowTimedPath 执行 → 在线记忆图 add_or_merge
（D4）→ is_found 推进（可见即推进，逐字保留）→ 卡死判据（上游 L534）升级
FAULT 收口。收口：landmark 全部达成 / max_steps 预算耗尽 / exploit 走完 →
STOPPED（task_complete）；连续 3 次 LLM/服务失败 → FAULT；cancel → 速度门控
→ STOPPED。

时间：结果绑定请求时各视角图像 stamp，max_result_age_s=90 s（D21）；图像
新鲜度用 stamp 匹配的有界定位历史（YOPO 教训），不与最新定位比较。
"""
import collections
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal
from uav_planner_interfaces.msg import TimedPath, TimedTrajectoryPoint

from .clip_client import ClipScoreClient
from .grounding_client import ClientError, GroundingClient, encode_png
from .hspm import OBS_VIEWPOINTS, extract_landmarks, is_stuck, plan_next_waypoint
from .llm_client import LlmQueryClient
from .memory import MemoryGraph
from .perception import rotation_body_from_optical
from .trajectory import build_timed_path, calculate_movement_steps

STATES = ("IDLE", "RUNNING", "STOPPING", "STOPPED", "FAULT")

# D13 同款收口码表（共享 NavigateToPose Result.code 词表，与 SPF 一致）
SHARED_CODES = {"task_complete": "SUCCEEDED", "cancelled": "CANCELED",
                "not_ready": "NOT_READY", "goal_timeout": "TIMEOUT",
                "llm_failures": "EXECUTION_FAILED",
                "service_failures": "EXECUTION_FAILED",
                "goal_rejected": "EXECUTION_FAILED",
                "segment_failed": "EXECUTION_FAILED",
                "stuck": "EXECUTION_FAILED"}


def image_to_array(msg: Image, dtype, channels: int) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=dtype)
    row = msg.width * channels
    step = msg.step if msg.step > 0 else row * buf.itemsize
    per_row = step // buf.itemsize
    if per_row == row:
        return buf.reshape(msg.height, msg.width, channels)
    return buf.reshape(msg.height, per_row)[:, :row].reshape(msg.height, msg.width, channels)


class CityNavPlannerNode(Node):
    def __init__(self):
        super().__init__("citynav_planner")
        self.declare_parameter("vehicle_namespace", "drone0")
        self.declare_parameter("view_cameras", ["LeftCamera", "SlightLeftCamera",
                                                "FrontCamera", "SlightRightCamera",
                                                "RightCamera"])
        self.declare_parameter("view_yaws_deg", [90.0, 45.0, 0.0, -45.0, -90.0])
        self.declare_parameter("base_velocity", 1.0)
        self.declare_parameter("base_yaw_rate", 30.0)
        self.declare_parameter("min_altitude_m", 1.2)
        self.declare_parameter("alpha", 0.6)
        self.declare_parameter("merge_radius", 20.0)
        self.declare_parameter("max_steps", 60)
        self.declare_parameter("view_freshness_s", 2.0)
        self.declare_parameter("max_result_age_s", 90.0)
        self.declare_parameter("llm_timeout_s", 30.0)
        self.declare_parameter("service_timeout_s", 30.0)
        self.declare_parameter("max_consecutive_failures", 3)
        self.declare_parameter("retry_backoff_s", 5.0)
        self.declare_parameter("pose_history_s", 5.0)
        self.declare_parameter("handoff_lead_s", 0.5)
        self.declare_parameter("grounding_url",
                               "http://127.0.0.1:8100/models/grounded_sam/greedy_mask_predict")
        self.declare_parameter("clip_url", "http://127.0.0.1:8101/models/clip/score")
        self.declare_parameter("memory_graph_path", "citynav_memory.json")

        ns = self.get_parameter("vehicle_namespace").value
        self._ns = ns
        views = list(self.get_parameter("view_cameras").value)
        yaws = [float(v) for v in self.get_parameter("view_yaws_deg").value]
        if len(views) != len(OBS_VIEWPOINTS) or len(yaws) != len(views):
            raise ValueError("view_cameras/view_yaws_deg must have 5 entries")
        self._view_names = dict(zip(views, OBS_VIEWPOINTS))
        self._view_rot = {self._view_names[v]: rotation_body_from_optical(y)
                          for v, y in zip(views, yaws)}
        self._cfg = {k: self.get_parameter(k).value for k in (
            "base_velocity", "base_yaw_rate", "min_altitude_m", "alpha",
            "merge_radius", "max_steps", "view_freshness_s", "llm_timeout_s",
            "service_timeout_s", "retry_backoff_s", "pose_history_s")}

        self._lock = threading.Lock()
        self._state = "IDLE"
        self._job = None
        self._stop_requested = False
        self._generation = 0
        self._revision = 0
        self._task_id = ""
        # 观测缓存：latest per view（stamp 为 AirSim 仿真时间，与 /clock 同源）
        self._rgb = {}      # view -> (stamp_s, stamp_ns, ndarray)
        self._depth = {}    # view -> (stamp_s, stamp_ns, ndarray)
        self._K = {}        # view -> 3x3
        self._pose_hist = collections.deque()  # (stamp, x, y, z, qx, qy, qz, qw)
        self._pose_latest = None
        self._twist_speed = None
        self._failures = 0

        self._landmarks = None
        self._next_landmark_idx = 0
        self._step_size = 0
        self._hist_step_size = collections.deque(maxlen=8)
        self._graph = None
        self._goal_result_event = threading.Event()
        self._goal_handle_accepted = threading.Event()
        self._goal_rejected = False
        self._goal_success = None
        self._plan_duration = 2.0
        self._feedback_revision = 0
        self._feedback_elapsed = 0.0

        # 话题/服务名相对节点命名空间（__ns:=/<vehicle_namespace>，SPF 同约定）
        for cam, view in self._view_names.items():
            base = f"sensor_measurements/{cam}"
            self.create_subscription(Image, f"{base}/rgb/image",
                                     lambda m, v=view: self._on_image(m, v, "rgb"),
                                     qos_profile_sensor_data)
            self.create_subscription(Image, f"{base}/depth_planar/image",
                                     lambda m, v=view: self._on_image(m, v, "depth"),
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f"{base}/rgb/camera_info",
                                     lambda m, v=view: self._on_camera_info(m, v),
                                     qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "self_localization/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "self_localization/twist",
                                 self._on_twist, qos_profile_sensor_data)
        self.create_subscription(String, "citynav/goal", self._on_goal, 10)
        self.create_service(Trigger, "citynav/cancel", self._on_cancel)
        self._status_pub = self.create_publisher(String, "citynav/status", 10)
        self._events_pub = self.create_publisher(String, "citynav/events", 10)

        self._action = ActionClient(self, FollowTimedPath, "follow_timed_path")
        self._modify = self.create_client(FollowTimedPath_SendGoal,
                                          "follow_timed_path/_behavior/modify")
        self._stopper = self.create_client(Trigger, "follow_timed_path/_behavior/stop")
        self._llm = LlmQueryClient(self, self.get_parameter("max_result_age_s").value)
        self._grounding = GroundingClient(self.get_parameter("grounding_url").value,
                                          timeout_s=self._cfg["service_timeout_s"])
        self._clip = ClipScoreClient(self.get_parameter("clip_url").value,
                                     timeout_s=self._cfg["service_timeout_s"])

        self._worker = threading.Thread(target=self._work_loop, daemon=True)
        self._worker.start()
        self.create_timer(0.5, self._publish_heartbeat)
        self._publish_status("ready", reason="init")

    # ---------- 输入回调 ----------
    def _on_image(self, msg: Image, view: str, kind: str):
        try:
            if kind == "rgb":
                arr = image_to_array(msg, np.uint8, 3)
            else:
                arr = image_to_array(msg, np.float32, 1)
        except ValueError:
            return
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            (self._rgb if kind == "rgb" else self._depth)[view] = (stamp, stamp_ns, arr)

    def _on_camera_info(self, msg: CameraInfo, view: str):
        k = msg.k
        if k[0] <= 0 or k[4] <= 0:
            return
        with self._lock:
            self._K[view] = np.array(k, dtype=np.float64).reshape(3, 3)

    def _on_pose(self, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        sample = (stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        with self._lock:
            self._pose_latest = sample
            self._pose_hist.append(sample)
            horizon = self._cfg["pose_history_s"]
            while self._pose_hist and stamp - self._pose_hist[0][0] > horizon:
                self._pose_hist.popleft()

    def _on_twist(self, msg: TwistStamped):
        v = msg.twist.linear
        with self._lock:
            self._twist_speed = (time.monotonic(), math.hypot(v.x, v.y))

    def _on_goal(self, msg: String):
        try:
            payload = json.loads(msg.data)
            instruction = str(payload["instruction"])
        except (ValueError, KeyError):
            self._publish_status("rejected", reason="bad_goal")
            return
        with self._lock:
            if self._state == "RUNNING":
                self._publish_status("rejected", reason="busy")
                return
            self._job = {"instruction": instruction,
                         "max_steps": int(payload.get("max_steps",
                                                      self._cfg["max_steps"])),
                         "use_memory": bool(payload.get("use_memory", True))}
            self._state = "RUNNING"
            self._generation += 1  # 作废上一任务的异步回调
            self._task_id = f"citynav-{int(time.time())}"
            self._revision = 0
            self._stop_requested = False
            self._failures = 0
            self._landmarks = None
            self._next_landmark_idx = 0
            self._step_size = 0
            self._hist_step_size.clear()
        self._init_memory()
        self._publish_event("goal_accepted", instruction=instruction)
        self._publish_status("started", instruction=instruction)

    def _on_cancel(self, request, response):
        with self._lock:
            if self._state in ("IDLE", "STOPPED"):
                response.success = True
                response.message = f"already {self._state}"
                return response
            self._generation += 1  # 作废推理与执行响应（yopo 同法）
            self._stop_requested = True
            self._state = "STOPPING"
        response.success = True
        response.message = "cancel initiated"
        return response

    def _init_memory(self):
        path = self.get_parameter("memory_graph_path").value
        self._graph = MemoryGraph(merge_radius=self._cfg["merge_radius"])
        if path and os.path.isfile(path):
            try:
                self._graph = MemoryGraph.load(path)
                self._graph.merge_radius = self._cfg["merge_radius"]
                self._publish_event("memory_loaded", path=path,
                                    nodes=self._graph.vert_count)
            except (ValueError, OSError) as exc:
                self.get_logger().warn("memory load failed: %s", exc)

    # ---------- 批次与定位 ----------
    def _collect_batch(self):
        """五目批次取齐：任一视角缺图/缺内参，或任一 stamp 超 view_freshness_s
        （以最新 stamp 为参考）→ 整批作废（契约 §4）。"""
        freshness = self._cfg["view_freshness_s"]
        with self._lock:
            rgbs, deps, ks = dict(self._rgb), dict(self._depth), dict(self._K)
        for view in OBS_VIEWPOINTS:
            if view not in rgbs or view not in deps or view not in ks:
                return None
        stamps = [rgbs[v][0] for v in OBS_VIEWPOINTS] + [deps[v][0] for v in OBS_VIEWPOINTS]
        ref = max(stamps)
        if any(s < ref - freshness for s in stamps):
            return None
        batch = []
        for view in OBS_VIEWPOINTS:
            stamp = rgbs[view][0]
            pose = self._pose_at(stamp)
            if pose is None:
                return None  # 有界定位历史无 stamp 匹配样本（YOPO 教训）
            batch.append({"view": view, "rgb": rgbs[view][2], "depth": deps[view][2],
                          "K": ks[view], "R": self._view_rot[view],
                          "pose": pose, "stamp": stamp,
                          "stamp_ns": rgbs[view][1]})
        return batch

    def _pose_at(self, stamp: float):
        """图像 stamp 匹配的有界定位历史样本（取 ≤stamp 的最新一条）。"""
        with self._lock:
            for sample in reversed(self._pose_hist):
                if sample[0] <= stamp + 0.05:
                    return np.array(sample[1:], dtype=np.float64)
        return None

    def _current_pose(self):
        """最新定位 → [x, y, z, yaw]（ENU；规划/分解只用 yaw，反投影走
        _pose_at 的完整四元数路径）。"""
        with self._lock:
            if not self._pose_latest:
                return None
        _, x, y, z, qx, qy, qz, qw = self._pose_latest
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return np.array([x, y, z, yaw], dtype=np.float64)

    # ---------- 主循环 ----------
    def _work_loop(self):
        while rclpy.ok():
            time.sleep(0.05)
            with self._lock:
                job, state = self._job, self._state
            if state not in ("RUNNING", "STOPPING"):
                continue
            if self._stop_requested:
                self._do_stop(final=False)
                continue
            self._run_one_cycle(job)

    def _run_one_cycle(self, job):
        if not self._ready():
            self._finish("STOPPED", "not_ready")
            return
        if self._landmarks is None:
            try:
                res = extract_landmarks(self._llm.request, job["instruction"],
                                        self._cfg["llm_timeout_s"])
            except ClientError as exc:
                res = {"ok": False, "error_code": f"service:{exc}"}
            if not res.get("ok"):
                self._count_failure("landmark_extract", res.get("error_code"))
                return
            self._landmarks = res["landmarks"]
            self._failures = 0
            self._publish_event("landmarks_extracted", landmarks=self._landmarks)
            return  # 下一循环开始探索

        batch = self._collect_batch()
        if batch is None:
            self._publish_status("batch_stale")
            time.sleep(0.5)
            return

        cur = self._current_pose()
        if cur is None:
            self._finish("STOPPED", "not_ready")
            return

        # exploit 优先（SimRun.py L452-492 顺序：先命中检查）
        if job["use_memory"] and self._graph and self._graph.vert_count > 0:
            cls_node = self._graph.closest_node(cur[:3])
            if cls_node is not None:
                self._run_exploit(job, cls_node, cur)
                return

        try:
            plan = plan_next_waypoint(
                self._llm.request, self._grounding.greedy_mask_predict,
                batch, job["instruction"], self._landmarks,
                self._next_landmark_idx, cur[:3],
                alpha=self._cfg["alpha"], min_altitude_m=self._cfg["min_altitude_m"],
                llm_timeout_s=self._cfg["llm_timeout_s"],
                max_age_s=float(self.get_parameter("max_result_age_s").value),
                logger=self.get_logger())
        except ClientError as exc:
            plan = {"ok": False, "error_code": f"grounding:{exc}"}
        if not plan.get("ok"):
            self._count_failure("hspm", plan.get("error_code"))
            return
        self._failures = 0

        # 在线记忆图记录（D4：每探索循环 add_or_merge，front 图入节点；
        # 五目阵无 back 相机，back 槽位留空，CLIP 评分跳过 None）
        node_tag = self._landmarks[self._next_landmark_idx] if plan["is_found"] else None
        self._graph.add_or_merge(cur[:3],
                                 images=(self._rgb_png(self._view_rgb(batch, "front")),
                                         None),
                                 text=node_tag)

        if plan["is_found"]:  # 可见即推进（上游 L257-258/L525-526 逐字语义）
            self._next_landmark_idx += 1
            self._publish_event("landmark_advanced", idx=self._next_landmark_idx,
                                landmark=self._landmarks[self._next_landmark_idx - 1])

        wp = plan["waypoint"]
        if wp is None:  # 分割全失败：原地（上游 route_coords=cur_pos），仅旋转分解
            wp = [float(cur[0]), float(cur[1]), float(cur[2]), float(cur[3])]
        n, _ = calculate_movement_steps(
            [cur[0], cur[1], cur[2], 0.0, 0.0, cur[3]], wp)
        timed = build_timed_path([wp], (cur[0], cur[1], cur[2], cur[3]),
                                 base_velocity=self._cfg["base_velocity"],
                                 base_yaw_rate=self._cfg["base_yaw_rate"],
                                 min_altitude_m=self._cfg["min_altitude_m"])
        self._step_size += n
        self._hist_step_size.append(n)
        self._publish_event("cycle", steps=n, step_size=self._step_size,
                            waypoint=wp, seg_success=plan["seg_success"],
                            is_found=plan["is_found"], selected=plan["selected"])

        if self._next_landmark_idx >= len(self._landmarks):
            self._save_memory()
            self._finish("STOPPED", "task_complete")
            return
        if is_stuck(list(self._hist_step_size)):  # 上游 L534 判据 → FAULT 收口
            self._save_memory()
            self._do_fault("stuck")
            return
        if self._step_size >= job["max_steps"]:
            self._save_memory()
            self._finish("STOPPED", "task_complete")
            return
        if timed is not None and self._send_path(timed) is None:
            return  # 失败已在 _send_path/_wait_segment 收口

    def _run_exploit(self, job, cls_node, cur):
        self._publish_event("exploit_hit", node=int(cls_node))
        remaining = self._landmarks[self._next_landmark_idx:]
        if not remaining:
            self._save_memory()
            self._finish("STOPPED", "task_complete")
            return
        try:
            sim = self._graph.nodes_landmarks_similarity(remaining, self._clip.score)
            route = self._graph.find_optimal_route(remaining, int(cls_node), sim,
                                                   alpha=1e-4)
        except ClientError as exc:
            self._count_failure("clip", f"clip:{exc}")
            return
        self._failures = 0
        waypoints = self._graph.exploit_waypoints(route["walk"], cur[:3])
        if not waypoints:
            self._save_memory()
            self._finish("STOPPED", "task_complete")
            return
        timed = build_timed_path(waypoints, (cur[0], cur[1], cur[2], cur[3]),
                                 base_velocity=self._cfg["base_velocity"],
                                 base_yaw_rate=self._cfg["base_yaw_rate"],
                                 min_altitude_m=self._cfg["min_altitude_m"])
        if timed is None:
            self._save_memory()
            self._finish("STOPPED", "task_complete")
            return
        self._step_size += timed["steps"]
        self._hist_step_size.append(timed["steps"])
        self._publish_event("exploit_executed", nodes=len(waypoints),
                            steps=timed["steps"])
        if self._send_path(timed) is None:
            return
        self._save_memory()
        self._finish("STOPPED", "task_complete")

    def _view_rgb(self, batch, view):
        for item in batch:
            if item["view"] == view:
                return item["rgb"]
        return np.zeros((1, 1, 3), np.uint8)

    @staticmethod
    def _rgb_png(rgb):
        return encode_png(np.ascontiguousarray(rgb))

    def _count_failure(self, stage, code):
        self._failures += 1
        self.get_logger().warn("%s failure %d: %s", stage, self._failures, code)
        self._publish_status(f"{stage}_failure", code=str(code),
                             failures=self._failures)
        if self._failures >= int(self.get_parameter("max_consecutive_failures").value):
            self._save_memory()
            self._do_fault("llm_failures" if stage in ("landmark_extract", "hspm")
                           else "service_failures")
            return
        time.sleep(self._cfg["retry_backoff_s"])

    # ---------- FollowTimedPath 执行 ----------
    def _ready(self):
        with self._lock:
            has_pose = bool(self._pose_latest)
        return has_pose and self._action.server_is_ready() \
            and self._modify.service_is_ready() and self._stopper.service_is_ready()

    def _send_path(self, timed):
        with self._lock:
            self._revision += 1
            revision = self._revision
            generation = self._generation
        path = self._make_timed_path(timed, revision, True)
        goal = FollowTimedPath.Goal()
        goal.path = path
        self._goal_handle_accepted.clear()
        self._goal_result_event.clear()
        self._goal_rejected = False
        self._goal_success = None
        future = self._action.send_goal_async(
            goal, feedback_callback=lambda m: self._on_feedback(m, generation))
        future.add_done_callback(lambda f: self._on_goal_response(f, generation))
        if not self._goal_handle_accepted.wait(15.0):
            self._finish("STOPPED", "goal_timeout")
            return None
        if self._goal_rejected:
            self._finish("STOPPED", "goal_rejected")
            return None
        self._plan_duration = timed["duration"]
        self._wait_segment(revision, generation)
        return revision

    def _on_goal_response(self, future, generation):
        if generation != self._generation:
            return
        handle = future.result()
        if handle.accepted:
            handle.get_result_async().add_done_callback(
                lambda f: self._on_goal_result(f, generation))
            self._goal_rejected = False
        else:
            self._goal_rejected = True
        self._goal_handle_accepted.set()

    def _on_goal_result(self, future, generation):
        if generation != self._generation:
            return
        # GoalResult status: 4 == SUCCEEDED (rclpy action goal status)
        self._goal_success = future.result().status == 4
        self._goal_result_event.set()

    def _on_feedback(self, msg, generation):
        if generation != self._generation:
            return
        self._feedback_revision = msg.feedback.revision
        self._feedback_elapsed = float(msg.feedback.elapsed)

    def _wait_segment(self, revision, generation):
        deadline = time.monotonic() + self._plan_duration + 30.0
        while time.monotonic() < deadline:
            with self._lock:
                if self._stop_requested or self._generation != generation \
                        or self._state != "RUNNING":
                    return
            if self._goal_result_event.wait(0.2):
                if self._goal_success:
                    return
                self._publish_status("segment_failed", revision=revision)
                self._finish("STOPPED", "segment_failed")
                return
            time.sleep(0.05)
        self._publish_status("segment_timeout", revision=revision)

    def _make_timed_path(self, timed, revision, is_final) -> TimedPath:
        now = self.get_clock().now().to_msg()
        # 行为时间线要求 valid_from 严格在未来且 generated_at<=now
        # （timeline.hpp validate）；小交接提前量覆盖 DDS 传输与处理延迟（SPF 同法）。
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
        for pt in timed["points"]:
            tp = TimedTrajectoryPoint()
            tp.time_from_start.sec = int(pt["t"])
            tp.time_from_start.nanosec = int(round((pt["t"] - int(pt["t"])) * 1e9))
            tp.position.x, tp.position.y, tp.position.z = pt["x"], pt["y"], pt["z"]
            tp.velocity.x, tp.velocity.y, tp.velocity.z = pt["vx"], pt["vy"], pt["vz"]
            tp.yaw = pt["yaw"]
            path.points.append(tp)
        return path

    # ---------- 收口 ----------
    def _do_stop(self, final, reason="cancelled"):
        try:
            self._stopper.call_async(Trigger.Request())
        except Exception as exc:
            self._publish_status("stop_call_failed", detail=str(exc))
        gate_deadline = time.monotonic() + 10.0
        stable = 0.0
        while time.monotonic() < gate_deadline:
            with self._lock:
                speed = self._twist_speed
            if speed and speed[1] < 0.2:
                stable += 0.1
                if stable >= 1.0:
                    self._save_memory()
                    self._finish("STOPPED", reason)
                    return
            else:
                stable = 0.0
            time.sleep(0.1)
        with self._lock:
            self._state = "FAULT"
        self._publish_status("fault", reason="STOP_UNCONFIRMED",
                             detail=f"{reason}_stop_gate_timeout",
                             stop_confirmed=False)

    def _do_fault(self, reason):
        try:
            self._stopper.call_async(Trigger.Request())
        except Exception as exc:
            self._publish_status("stop_call_failed", detail=str(exc))
        with self._lock:
            self._state = "FAULT"
            self._job = None
            self._stop_requested = False
        self._publish_status("fault", reason=SHARED_CODES.get(reason, "EXECUTION_FAILED"),
                             detail=reason, stop_confirmed=None)

    def _finish(self, state, reason):
        with self._lock:
            step_size = self._step_size
            self._state = state
            self._job = None
            self._stop_requested = False
        self._publish_status(state, reason=SHARED_CODES.get(reason, "EXECUTION_FAILED"),
                             detail=reason, step_size=step_size,
                             stop_confirmed=(state == "STOPPED"))

    def _save_memory(self):
        path = self.get_parameter("memory_graph_path").value
        if path and self._graph is not None and self._graph.vert_count > 0:
            try:
                self._graph.save(path)
                self._publish_event("memory_saved", path=path,
                                   nodes=self._graph.vert_count)
            except OSError as exc:
                self.get_logger().warn("memory save failed: %s", exc)

    # ---------- 状态/事件 ----------
    def _status_snapshot(self, state):
        with self._lock:
            revision, task_id = self._revision, self._task_id
            generation = self._generation
            landmarks = None if self._landmarks is None else len(self._landmarks)
            nodes = self._graph.vert_count if self._graph else 0
        return {"state": state, "task_id": task_id, "revision": revision,
                "generation": generation, "ready": self._ready(),
                "step_size": self._step_size,
                "next_landmark_idx": self._next_landmark_idx,
                "landmarks": landmarks, "memory_nodes": nodes,
                "failures": self._failures}

    def _publish_status(self, state, **extra):
        payload = self._status_snapshot(state)
        payload.update(extra)
        self._status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_event(self, event, **extra):
        payload = {"event": event, "task_id": self._task_id,
                   "sim_time": self.get_clock().now().to_msg().sec}
        payload.update(extra)
        self._events_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_heartbeat(self):
        with self._lock:
            state = self._state
        self._publish_status(state)


def main():
    rclpy.init()
    node = CityNavPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
