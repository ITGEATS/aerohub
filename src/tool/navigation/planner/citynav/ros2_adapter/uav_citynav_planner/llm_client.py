"""citynav/llm/query → citynav/llm/result 轻量客户端（板端纯 stdlib+rclpy）。

权威契约（uav_citynav_llm_proxy node.py L13-17 定型，本客户端逐字段对齐）：
  query（std_msgs/String JSON）：
    {"request_id", "op": "viewpoint"|"landmark"|"caption", "instruction",
     "landmarks", "traversed", "next", "view_stamps": {label: ns}, "max_age_s"}
    ——prompt 与图像都不由 planner 发送；代理按 view_stamps 从自身相机缓存
    取帧并内部构造上游消息。
  result：{"request_id", "op", "ok", "text", "error_code", "latency_s"}；
    error_code ∈ ok|stale|timeout|http|parse|busy。

request_id 匹配、过期丢弃（本地墙钟 max_result_age_s=90 s，契约 §4 D21）、
无结果超时。代理单飞 busy 立即返回，调用方按 error_code 退避重试；上游无限
递归重试不迁移（D10），连续失败计数由 node 收口 FAULT。
"""
import json
import threading
import time
import uuid

from std_msgs.msg import String

ERROR_CODES = ("ok", "stale", "timeout", "http", "parse", "busy")


class LlmQueryClient:
    def __init__(self, node, max_result_age_s: float = 90.0):
        self._node = node
        self._max_age = max_result_age_s
        self._lock = threading.Lock()
        self._pending = None  # (request_id, submit_wall, result dict | None)
        # Relative to the node namespace (__ns:=/<vehicle_namespace>).
        self._pub = node.create_publisher(String, "citynav/llm/query", 10)
        node.create_subscription(String, "citynav/llm/result",
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

    def request(self, op: str, timeout_s: float, **payload):
        """阻塞式一次 LLM 查询；payload 为权威契约字段（不含 request_id/op）。

        timeout_s 仅作本地等待包络，不上 wire。返回 result dict；失败
        {"ok": False, "error_code": ...}。
        """
        request_id = uuid.uuid4().hex
        query = {"request_id": request_id, "op": op}
        query.update(payload)
        with self._lock:
            self._pending = (request_id, time.monotonic(), None)
        self._pub.publish(String(data=json.dumps(query, ensure_ascii=False)))
        deadline = time.monotonic() + timeout_s + self._max_age
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
                return {"ok": False, "error_code": "timeout"}
            if time.monotonic() - got.get("_wall", time.monotonic()) > self._max_age:
                return {"ok": False, "error_code": "stale"}
            return got
        with self._lock:
            self._pending = None
        return {"ok": False, "error_code": "timeout"}
