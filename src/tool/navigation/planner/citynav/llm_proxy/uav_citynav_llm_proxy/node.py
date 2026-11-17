"""uav_citynav_llm_proxy：CityNavAgent 云端 LLM 代理（工作站侧，迁移方案阶段 3）。

数据流（README §3 最小调用链）：
  算力容器 uav_citynav_planner --query(JSON,无图像)--> 本节点 --HTTPS--> 云端
  glm-4v-flash；本节点订阅五目相机阵 rgb，缓存各 view 最新帧（保留采集 stamp），
  按 query.view_stamps 对齐取帧，构造上游同构消息（http_client.py），result
  回发模型原始文本；op=viewpoint 时代理侧先跑 parse_viewpoint_response_v2 校验
  JSON 可解析（error_code=parse），planner 侧仍可自行解析取字段。

话题（相对节点 namespace，启动须 __ns:=/<vehicle_namespace>，与 SPF 代理同约定；
传感器桥发布前缀即 /<vehicle_namespace>/sensor_measurements，故相机话题同为相对）：
  入 sensor_measurements/<Cam>/rgb/image  sensor_msgs/Image（rgb8，sensor-data QoS）
  入 citynav/llm/query   std_msgs/String  {"request_id","op":"viewpoint"|"landmark"|
      "caption","instruction","landmarks","traversed","next",
      "view_stamps":{label:ns},"max_age_s"}
  出 citynav/llm/result  std_msgs/String  {"request_id","op","ok","text",
      "error_code","latency_s"}；error_code ∈ ok|stale|timeout|http|parse|busy

参数：use_sim_time（rclpy 内建，时钟/帧龄按仿真时间）、vehicle_namespace（仅日志
与启动约定）、cameras（"label=camera" 字符串列表，顺序=上游 obs_viewpoint 顺序）、
model/base_url/api_key_env（默认 glm-4v-flash / 智谱 / OPENAI_API_KEY，配置化）、
cache_dir（空=禁用磁盘缓存）、request_timeout_s。

失败路径：stale（缺帧或帧龄超 max_age_s，默认 90 s=D21 包络）/timeout/http
（传输，见 http_client.py）/parse（query JSON 坏或 viewpoint 响应不可解析）/
busy（单飞：处理中收到新 query 立即回 busy，不排队）。
"""
import json
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from . import http_client as hc
from .prompts import (landmark_caption_prompt_builder,
                      landmark_extraction_messages,
                      parse_viewpoint_response_v2,
                      route_planning_prompt_builder)

# view 标签→相机名默认五目固定阵（契约 §5/D8'；顺序=上游 obs_viewpoint）
DEFAULT_CAMERAS = [
    "left=CityLeft",
    "slightly_left=CitySlightlyLeft",
    "front=CityFront",
    "slightly_right=CitySlightlyRight",
    "right=CityRight",
]

# 契约 §5/D21：结果绑定请求时图像 stamp 的时间包络（LLM ~10 s + grounding 串行）
DEFAULT_MAX_AGE_S = 90.0


class CitynavLlmProxyNode(Node):
    def __init__(self):
        super().__init__("citynav_llm_proxy")
        self.declare_parameter("vehicle_namespace", "drone0")
        self.declare_parameter("cameras", DEFAULT_CAMERAS)
        self.declare_parameter("model", hc.DEFAULT_MODEL)
        self.declare_parameter("base_url", hc.DEFAULT_BASE_URL)
        self.declare_parameter("api_key_env", hc.DEFAULT_API_KEY_ENV)
        self.declare_parameter("cache_dir", "")
        self.declare_parameter("request_timeout_s", 30.0)

        self._ns = self.get_parameter("vehicle_namespace").value
        try:
            self._views = hc.parse_cameras(
                self.get_parameter("cameras").value)
        except ValueError as exc:
            self.get_logger().error(f"bad cameras param: {exc}")
            raise
        self._model = self.get_parameter("model").value
        self._base_url = self.get_parameter("base_url").value
        self._api_key_env = self.get_parameter("api_key_env").value
        self._cache_dir = self.get_parameter("cache_dir").value
        self._timeout = float(self.get_parameter("request_timeout_s").value)

        self._frames_lock = threading.Lock()
        self._frames = {}  # label -> (stamp_ns, png_b64)
        self._busy_lock = threading.Lock()
        self._busy = False
        self._jobs = queue.Queue()
        self._running = True

        for label, cam in self._views.items():
            self.create_subscription(
                Image, f"sensor_measurements/{cam}/rgb/image",
                self._make_image_cb(label), qos_profile_sensor_data)
        self.create_subscription(String, "citynav/llm/query",
                                 self._on_query, 10)
        self._result_pub = self.create_publisher(String, "citynav/llm/result", 10)

        self._worker = threading.Thread(target=self._work_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"CityNav LLM proxy ready: model={self._model} "
            f"base_url={self._base_url} ns={self._ns} "
            f"views={list(self._views)} cache_dir={self._cache_dir or '(off)'}")

    # ---------- 图像缓存 ----------

    def _make_image_cb(self, label):
        def cb(msg):
            self._on_image(label, msg)
        return cb

    def _on_image(self, label, msg):
        if msg.encoding != "rgb8":
            return
        try:
            b64 = hc.b64_png(msg.width, msg.height, msg.data, msg.step)
        except ValueError:
            return
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        with self._frames_lock:
            self._frames[label] = (stamp_ns, b64)

    # ---------- query 入口（单飞 busy） ----------

    def _on_query(self, msg):
        try:
            query = json.loads(msg.data)
            request_id = str(query["request_id"])
        except (ValueError, KeyError) as exc:
            self.get_logger().warn(f"bad query payload: {exc}")
            return
        with self._busy_lock:
            if self._busy:
                self._publish({"request_id": request_id,
                               "op": str(query.get("op", "")),
                               "ok": False, "text": "",
                               "error_code": "busy", "latency_s": 0.0})
                return
            self._busy = True
        self._jobs.put(query)

    def _work_loop(self):
        while self._running:
            try:
                query = self._jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._handle(query)
            finally:
                with self._busy_lock:
                    self._busy = False

    # ---------- 处理 ----------

    def _handle(self, query):
        res = {"request_id": str(query.get("request_id", "")),
               "op": str(query.get("op", "")),
               "ok": False, "text": "", "error_code": "parse"}
        t0 = time.monotonic()
        try:
            self._process(query, res)
        except hc.LlmHttpError as exc:
            res["error_code"] = exc.code  # http | timeout
            self.get_logger().warn(f"llm http failure [{res['error_code']}] {exc.detail}")
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"bad query fields: {exc}")
        except Exception as exc:  # 兜底：worker 永不退出
            res["error_code"] = "http"
            self.get_logger().error(f"unexpected: {exc!r}")
        res["latency_s"] = round(time.monotonic() - t0, 3)
        self._publish(res)

    def _process(self, query, res):
        op = res["op"]
        if op == "landmark":
            messages = landmark_extraction_messages(str(query["instruction"]))
            cache_prompt = messages[0]["content"] + "\n" + messages[1]["content"]
        elif op in ("viewpoint", "caption"):
            images, stale = self._select_images(query)
            if stale is not None or not images:
                res["error_code"] = "stale"
                self.get_logger().warn(f"stale frame: {stale or 'no view_stamps'}")
                return
            if op == "viewpoint":
                prompt = route_planning_prompt_builder(
                    str(query["instruction"]),
                    list(query.get("landmarks") or []),
                    list(query.get("traversed") or []),
                    str(query.get("next", "")))
                content = hc.build_viewpoint_content(prompt, images)
            else:
                prompt = landmark_caption_prompt_builder(
                    list(query.get("landmarks") or []))
                content = hc.build_caption_content(prompt, images[0][1])
            cache_prompt = prompt
            messages = [{"role": "user", "content": content}]
        else:
            return  # error_code 保持 parse

        text = hc.chat(messages, model=self._model, base_url=self._base_url,
                       api_key=os.environ.get(self._api_key_env, ""),
                       timeout_s=self._timeout, cache_prompt=cache_prompt,
                       cache_dir=self._cache_dir)
        res["text"] = text
        if op == "viewpoint":
            try:
                parse_viewpoint_response_v2(text)
            except (ValueError, TypeError) as exc:
                res["error_code"] = "parse"
                self.get_logger().warn(f"viewpoint response not parseable: {exc}")
                return
        res["ok"] = True
        res["error_code"] = "ok"

    def _select_images(self, query):
        view_stamps = dict(query.get("view_stamps") or {})
        max_age_s = float(query.get("max_age_s", DEFAULT_MAX_AGE_S))
        now_ns = int(self.get_clock().now().nanoseconds)
        order = [label for label in self._views if label in view_stamps]
        with self._frames_lock:
            frames = dict(self._frames)
        return hc.select_frame(frames, view_stamps, order, now_ns,
                               int(max_age_s * 1e9))

    def _publish(self, res):
        self._result_pub.publish(String(data=json.dumps(res, ensure_ascii=False)))


def main():
    rclpy.init()
    node = CitynavLlmProxyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.try_shutdown()
