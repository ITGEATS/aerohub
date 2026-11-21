"""共享 Inference Function 的 CityNav 消费端（Tool 化改造 2026-11-18）。

原私有代理话题契约（citynav/llm/query|result，uav_citynav_llm_proxy）已按
"被复现工作的形式是 Tool"原则移除：prompt 构造留在本 Tool（prompts.py 逐字
复制上游），帧由 Tool 随请求显式提供（batch 内同一批图，与反投影共用——
较原代理"代理侧最新帧"更强，消除采集竞态），执行走共享
`uav_inference_interfaces/action/InferVisionLanguageFrames`（model_name 为
逻辑名，供应商/密钥/引擎是部署私有绑定，SPF vlm_client 同法）。

op 语义：
  landmark  —— system+user 文本抽取（images 空，纯文本请求）；
  viewpoint —— 五目帧（label=视角名，顺序=上游 obs_viewpoint）+ route prompt。
调用签名 request(op, timeout_s, **payload) 与原话题客户端保持一致，hspm 编排
不变；返回 {"ok","text","error_code","latency_s"}。上游无限递归重试不迁移
（D10），连续失败计数由 node 收口 FAULT。
"""
import time

import numpy as np
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from uav_inference_interfaces.action import InferVisionLanguageFrames

from . import prompts


def rgb_to_image(arr: np.ndarray, stamp_ns: int, frame_id: str) -> Image:
    """uint8 HxWx3 rgb → sensor_msgs/Image rgb8（无 cv2，纯 buffer 拷贝）。"""
    arr = np.ascontiguousarray(arr, dtype=np.uint8)
    if arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError("expected rgb uint8 HxWx3")
    h, w = arr.shape[:2]
    msg = Image()
    msg.header.stamp.sec = int(stamp_ns) // 10**9
    msg.header.stamp.nanosec = int(stamp_ns) % 10**9
    msg.header.frame_id = frame_id
    msg.height, msg.width = h, w
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = w * 3
    msg.data = arr.tobytes()
    return msg


class LlmFramesClient:
    def __init__(self, node, model_name: str, max_result_age_s: float = 90.0,
                 action_name: str = "inference/vision_language_frames/infer"):
        self._node = node
        self._model_name = model_name
        self._max_age = float(max_result_age_s)
        self._client = ActionClient(node, InferVisionLanguageFrames, action_name)

    @staticmethod
    def _cancel_late(future):
        try:
            handle = future.result()
            if handle.accepted:
                handle.cancel_goal_async()
        except Exception:
            pass

    def request(self, op: str, timeout_s: float, **payload):
        """阻塞式一次推理请求（worker 线程内调用，模式对齐 SPF VlmQueryClient）。

        返回 {"ok": bool, "text": str, "error_code": str, "latency_s": float}；
        失败 error_code 取 Function 端词表（NOT_READY/STALE_INPUT/TIMEOUT/
        CANCELED/BAD_PAYLOAD/UNSUPPORTED_MODEL/RATE_LIMITED/AUTH_ERROR/
        TRANSPORT_ERROR/STALE_RESULT）或本地 BUSY_OR_INVALID/NO_SERVER/
        interrupted。
        """
        if op == "landmark":
            prompt = prompts.landmark_extraction_user_prompt(
                str(payload["instruction"]))
            system = prompts.LANDMARK_EXTRACTION_SYSTEM_PROMPT
            frames = []
        elif op == "viewpoint":
            prompt = prompts.route_planning_prompt_builder(
                str(payload["instruction"]), list(payload["landmarks"]),
                list(payload.get("traversed") or []), str(payload["next"]))
            system = ""
            frames = list(payload.get("images") or [])
        else:
            return {"ok": False, "error_code": "bad_op", "text": "",
                    "latency_s": 0.0}
        if not self._client.server_is_ready():
            return {"ok": False, "error_code": "NO_SERVER", "text": "",
                    "latency_s": 0.0}
        goal = InferVisionLanguageFrames.Goal()
        goal.model_name = self._model_name
        goal.prompt = prompt
        goal.system_prompt = system
        for label, rgb, stamp_ns in frames:
            goal.image_labels.append(str(label))
            goal.images.append(rgb_to_image(
                rgb, int(stamp_ns),
                "%s/%s_image" % (self._node.get_namespace().strip("/"), label)))
        goal.timeout = float(timeout_s)
        goal.max_result_age = self._max_age
        started = time.monotonic()
        # Safety envelope beyond the Function's own budget enforcement (the
        # Function returns TIMEOUT at its deadline; this only guards against
        # a wedged server never answering at all).
        deadline = started + float(timeout_s) + 30.0
        acceptance = self._client.send_goal_async(goal)
        while not acceptance.done():
            if not self._node.context.ok() or time.monotonic() > deadline:
                acceptance.add_done_callback(self._cancel_late)
                return {"ok": False, "error_code": "interrupted", "text": "",
                        "latency_s": time.monotonic() - started}
            time.sleep(0.025)
        handle = acceptance.result()
        if handle is None or not handle.accepted:
            return {"ok": False, "error_code": "BUSY_OR_INVALID", "text": "",
                    "latency_s": time.monotonic() - started}
        future = handle.get_result_async()
        while not future.done():
            if not self._node.context.ok():
                handle.cancel_goal_async()
                return {"ok": False, "error_code": "interrupted", "text": "",
                        "latency_s": time.monotonic() - started}
            if time.monotonic() > deadline:
                handle.cancel_goal_async()
                return {"ok": False, "error_code": "LOCAL_TIMEOUT", "text": "",
                        "latency_s": time.monotonic() - started}
            time.sleep(0.025)
        wrapped = future.result()
        result = wrapped.result if wrapped is not None else None
        latency = time.monotonic() - started
        if result is None:
            return {"ok": False, "error_code": "NO_RESULT", "text": "",
                    "latency_s": latency}
        if not result.success:
            return {"ok": False, "error_code": result.error_code or "NO_RESULT",
                    "text": result.text or "", "latency_s": latency}
        return {"ok": True, "error_code": "ok", "text": result.text,
                "latency_s": latency}
