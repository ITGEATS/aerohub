"""SPF consumer of the generic Inference Function; no vendor HTTP or credentials.

The navigation prompt and interpretation of point/depth belong to this Tool.
Only model_name is selected here; the Function owns camera acquisition/encoding
and the private CUDA/Houmo/Cloud binding.
"""
import time

from rclpy.action import ActionClient
from uav_inference_interfaces.action import InferVisionLanguage

from .prompt import VlmParseError, build_prompt, parse_point_response

# SPF waits for inference between completed motion segments.
RESULT_MAX_AGE_S = 30.0


class VlmQueryClient:
    def __init__(self, node, model_name):
        self._node = node
        self._model_name = model_name
        self._client = ActionClient(node, InferVisionLanguage, 'inference/vision_language/infer')

    @staticmethod
    def _cancel_late(future):
        try:
            handle = future.result()
            if handle.accepted:
                handle.cancel_goal_async()
        except Exception:
            pass

    def request(self, instruction, timeout_s, interrupted=lambda: False):
        if not self._client.server_is_ready():
            return dict(ok=False, error_code='NOT_READY')
        request = InferVisionLanguage.Goal(
            model_name=self._model_name, prompt=build_prompt(instruction),
            timeout=float(timeout_s), max_result_age=RESULT_MAX_AGE_S)
        deadline = time.monotonic() + timeout_s
        acceptance = self._client.send_goal_async(request)
        while not acceptance.done():
            if interrupted() or time.monotonic() >= deadline:
                acceptance.add_done_callback(self._cancel_late)
                return dict(ok=False, error_code='interrupted')
            time.sleep(0.025)
        handle = acceptance.result()
        if not handle.accepted:
            return dict(ok=False, error_code='BUSY_OR_INVALID')
        future = handle.get_result_async()
        while not future.done():
            if interrupted() or time.monotonic() >= deadline:
                handle.cancel_goal_async()
                return dict(ok=False, error_code='interrupted')
            time.sleep(0.025)
        result = future.result().result
        if not result.success:
            return dict(ok=False, error_code=result.error_code)
        stamp = result.observation_header.stamp
        age = (self._node.get_clock().now().nanoseconds / 1e9
               - stamp.sec - stamp.nanosec * 1e-9)
        if age < -0.05 or age > RESULT_MAX_AGE_S:
            return dict(ok=False, error_code='STALE_RESULT')
        try:
            parsed = parse_point_response(result.text)
        except VlmParseError as error:
            return dict(ok=False, error_code=error.code)
        except (ValueError, TypeError, KeyError, IndexError, AttributeError):
            return dict(ok=False, error_code='bad_json')
        return dict(ok=True, **parsed, image_stamp=stamp.sec + stamp.nanosec * 1e-9,
                    model=result.model_name, latency_ms=result.latency_ms)
