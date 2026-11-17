"""spf/vlm/query → spf/vlm/result 轻量客户端（板端纯 stdlib+rclpy）。

request_id 匹配、过期丢弃（max_result_age_s）、无结果超时。图像不落板端：
query 只携带指令与请求时的图像 stamp（对齐用），result 由代理回发其实际帧 stamp。
"""
import json
import threading
import time
import uuid

from std_msgs.msg import String


class VlmQueryClient:
    def __init__(self, node, max_result_age_s: float = 6.0):
        self._node = node
        self._max_age = max_result_age_s
        self._lock = threading.Lock()
        self._pending = None  # (request_id, submit_wall, result dict | None)
        # Relative to the node namespace (__ns:=/<vehicle_namespace>).
        self._pub = node.create_publisher(String, "spf/vlm/query", 10)
        node.create_subscription(String, "spf/vlm/result",
                                 self._on_result, 10)

    def _on_result(self, msg):
        try:
            result = json.loads(msg.data)
        except ValueError:
            return
        result["_wall"] = time.monotonic()
        with self._lock:
            if self._pending and result.get("request_id") == self._pending[0]:
                self._pending = (self._pending[0], self._pending[1], result)

    def request(self, instruction: str, image_stamp: float, timeout_s: float):
        """阻塞式一次标注请求。返回 result dict；失败返回 {"ok": False, "error_code": ...}。"""
        request_id = uuid.uuid4().hex
        query = String(data=json.dumps({
            "request_id": request_id, "instruction": instruction,
            "image_stamp": image_stamp}))
        with self._lock:
            self._pending = (request_id, time.monotonic(), None)
        self._pub.publish(query)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                got = "pending" if (self._pending and self._pending[2] is None) else \
                    (self._pending[2] if self._pending else None)
            if got == "pending":
                time.sleep(0.05)
                continue
            with self._lock:
                self._pending = None
            if got is None:
                return {"ok": False, "error_code": "no_result"}
            if time.monotonic() - got.get("_wall", time.monotonic()) > self._max_age:
                return {"ok": False, "error_code": "stale_result"}
            return got
        with self._lock:
            self._pending = None
        return {"ok": False, "error_code": "query_timeout"}
