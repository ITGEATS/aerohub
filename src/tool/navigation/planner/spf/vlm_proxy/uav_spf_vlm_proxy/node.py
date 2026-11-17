"""uav_spf_vlm_proxy：SPF 云端 VLM 标注代理（PC 工作站侧，迁移方案阶段 3）。

数据流（方案 B）：
  板端 planner --query(JSON,无图像)--> 本节点 --HTTPS--> 云端 VLM
  本节点订阅传感器桥图像并缓存最新帧；query 的 image_stamp 仅作对齐校验，
  实际使用缓存的最新帧并在 result 中回发其真实 stamp。
  图像不过板、公网不出 PC（阶段 6 网络边界）。

接口（相对节点 namespace，默认 drone0）：
  入 sensor_measurements/FrontCamera/rgb/image  sensor_msgs/Image (rgb8, sensor-data QoS)
  入 spf/vlm/query  std_msgs/String  {"request_id","instruction","image_stamp":float}
  出 spf/vlm/result std_msgs/String  {"request_id","ok","image_stamp","point":[y,x],
      "depth","label","raw_text","latency_ms","model","error_code","error_detail"}

失败路径 error_code：empty_response/bad_json/no_points（解析，见 prompt.py）、
http_error/timeout/transport_error/bad_payload（传输，见 http_client.py）、
no_frame（无缓存图像）、bad_query。
"""
import base64
import json
import queue
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .http_client import VlmHttpError, request_point_text
from .prompt import VlmParseError, build_prompt, parse_point_response

# 上游 provider 默认模型映射（vlm_client._determine_model_name 语义）
UPSTREAM_DEFAULT_MODELS = {
    "openai": "google/gemini-2.5-flash",
    "gemini": "gemini-2.5-flash",
}


class SpfVlmProxyNode(Node):
    def __init__(self):
        super().__init__("spf_vlm_proxy")
        self.declare_parameter("provider", "openai")
        self.declare_parameter("model", "")          # 空 → provider 上游默认
        self.declare_parameter("api_key_env", "OPENAI_API_KEY")
        self.declare_parameter("base_url", "https://openrouter.ai/api/v1")
        self.declare_parameter("temperature", 0.4)   # 上游固定值
        self.declare_parameter("max_tokens", 1024)   # 智谱上限适配；论文模型可设 8192
        self.declare_parameter("request_timeout_s", 30.0)
        self.declare_parameter("image_scale", 1.0)   # 0.5 降采样选项（D4，归一化坐标几何等价）
        self.declare_parameter("camera_topic", "sensor_measurements/FrontCamera/rgb/image")
        self.declare_parameter("vehicle_namespace", "drone0")

        ns = self.get_parameter("vehicle_namespace").value
        self._provider = self.get_parameter("provider").value
        self._model = self.get_parameter("model").value or UPSTREAM_DEFAULT_MODELS[self._provider]
        self._api_key_env = self.get_parameter("api_key_env").value
        self._base_url = self.get_parameter("base_url").value
        self._temperature = float(self.get_parameter("temperature").value)
        self._max_tokens = int(self.get_parameter("max_tokens").value)
        self._timeout = float(self.get_parameter("request_timeout_s").value)
        self._image_scale = float(self.get_parameter("image_scale").value)

        self._frame_lock = threading.Lock()
        self._frame = None  # (stamp_sec, jpeg_b64)
        self._jobs = queue.Queue()
        self._running = True

        # Relative to the node namespace (__ns:=/<vehicle_namespace>), matching the
        # planner adapter; hard-coding the ns would double-prefix the topics.
        self.create_subscription(
            Image, self.get_parameter("camera_topic").value,
            self._on_image, qos_profile_sensor_data)
        self._query_sub = self.create_subscription(
            String, "spf/vlm/query", self._on_query, 10)
        self._result_pub = self.create_publisher(
            String, "spf/vlm/result", 10)

        self._worker = threading.Thread(target=self._work_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"SPF VLM proxy ready: provider={self._provider} model={self._model} "
            f"base_url={self._base_url} ns={ns}")

    def _stamp_sec(self, msg: Image) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_image(self, msg: Image):
        if msg.encoding != "rgb8":
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = arr.reshape(msg.height, msg.width, len(msg.data) // (msg.height * msg.width))[:, :, :3]
        except ValueError:
            return
        if self._image_scale != 1.0:
            img = cv2.resize(img, None, fx=self._image_scale, fy=self._image_scale,
                             interpolation=cv2.INTER_AREA)
        # 上游 cv2.imencode('.jpg') 默认质量；RGB→BGR 后编码保持颜色正确
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        if not ok:
            return
        with self._frame_lock:
            self._frame = (self._stamp_sec(msg), base64.b64encode(buf.tobytes()).decode())

    def _on_query(self, msg: String):
        try:
            query = json.loads(msg.data)
            request_id = str(query["request_id"])
            instruction = str(query["instruction"])
        except (ValueError, KeyError) as exc:
            self.get_logger().warn(f"bad query payload: {exc}")
            return
        self._jobs.put((request_id, instruction))

    def _work_loop(self):
        while self._running:
            try:
                request_id, instruction = self._jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            t0 = time.monotonic()
            with self._frame_lock:
                frame = self._frame
            result = {"request_id": request_id, "ok": False, "model": self._model}
            if frame is None:
                result.update(error_code="no_frame", error_detail="no cached image yet")
            else:
                stamp_sec, jpeg_b64 = frame
                result["image_stamp"] = stamp_sec
                try:
                    raw = request_point_text(
                        self._provider, self._api_key(), self._base_url, self._model,
                        build_prompt(instruction), jpeg_b64,
                        self._temperature, self._max_tokens, self._timeout)
                    result["raw_text"] = raw
                    try:
                        parsed = parse_point_response(raw)
                        result.update(ok=True, **parsed)
                    except VlmParseError as exc:
                        result.update(error_code=exc.code, error_detail=exc.detail)
                except VlmHttpError as exc:
                    result.update(error_code=exc.code, error_detail=exc.detail)
                except Exception as exc:  # 兜底：worker 永不退出
                    result.update(error_code="transport_error", error_detail=repr(exc))
            result["latency_ms"] = int((time.monotonic() - t0) * 1000)
            self._result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))

    def _api_key(self) -> str:
        import os
        return os.environ.get(self._api_key_env, "")


def main():
    rclpy.init()
    node = SpfVlmProxyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.try_shutdown()
