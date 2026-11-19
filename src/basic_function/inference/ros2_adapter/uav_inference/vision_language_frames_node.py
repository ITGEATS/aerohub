"""Frames variant of the vision-language Function: caller-supplied ordered
frames (multi-eye rigs / cached batches) or a text-only request.

No camera subscription and no navigation prompts here: the calling Tool owns
frame selection and prompt text; this node owns validation, JPEG encoding,
freshness checks against each frame header, single-flight busy semantics and
the private worker routing (same error vocabulary as the single-image node).
"""
import math
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from uav_inference_interfaces.action import InferVisionLanguageFrames

from .model_client import ModelClient, WorkerError
from .vision_language_node import jpeg_image
from .executor_shutdown import shutdown_executor

MAX_FRAMES = 8


def _validate(request):
    if (not request.model_name.strip() or not request.prompt.strip()
            or len(request.model_name) > 128 or len(request.prompt) > 65536
            or not all(math.isfinite(v) and v > 0
                       for v in (request.timeout, request.max_result_age))):
        return False
    if len(request.images) > MAX_FRAMES \
            or len(request.image_labels) != len(request.images):
        return False
    for image in request.images:
        if image.encoding not in ('rgb8', 'bgr8') \
                or not 0 < image.width <= 4096 or not 0 < image.height <= 4096 \
                or image.step < image.width * 3 \
                or len(image.data) != image.step * image.height \
                or not image.header.frame_id:
            return False
    return True


class VisionLanguageFramesNode(Node):
    def __init__(self):
        super().__init__('vision_language_frames_inference')
        self.endpoint = self.declare_parameter('endpoint', 'http://127.0.0.1:8013').value
        self._lock = threading.Lock()
        self._busy = False
        self._threads = set()
        self.server = ActionServer(
            self, InferVisionLanguageFrames,
            'inference/vision_language_frames/infer',
            goal_callback=self._goal, cancel_callback=lambda handle: CancelResponse.ACCEPT,
            execute_callback=self._execute, callback_group=ReentrantCallbackGroup())

    def _goal(self, request):
        if not _validate(request):
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _release(self):
        with self._lock:
            self._busy = False
            self._threads.discard(threading.current_thread())

    def _execute(self, handle):
        request = handle.request
        result = InferVisionLanguageFrames.Result(model_name=request.model_name)
        started = time.monotonic()
        deadline = started + request.timeout
        now_ns = self.get_clock().now().nanoseconds
        images = []
        try:
            for label, msg in zip(request.image_labels, request.images):
                stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                age = (now_ns - stamp_ns) / 1e9
                if age < -0.05 or age > request.max_result_age:
                    self._release()
                    result.error_code = 'STALE_INPUT'
                    result.message = 'frame %s is stale/future' % msg.header.frame_id
                    result.observation_headers = [m.header for m in request.images]
                    handle.abort()
                    return result
                images.append(dict(label=str(label), image_base64=jpeg_image(msg),
                                   stamp_ns=stamp_ns, frame=msg.header.frame_id))
        except ValueError:
            self._release()
            result.error_code = 'INVALID_INPUT'
            result.message = 'image conversion failed'
            handle.abort()
            return result
        result.observation_headers = [m.header for m in request.images]
        finished = threading.Event()
        output = {}
        request_id = uuid.uuid4().hex

        def work():
            try:
                if handle.is_cancel_requested:
                    raise WorkerError('CANCELED', 'caller canceled before inference')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise WorkerError('TIMEOUT', 'preprocessing exceeded request budget')
                client = ModelClient(self.endpoint, remaining)
                metadata = client.request('/health/ready')
                if request.model_name not in metadata.get('model_names', []):
                    raise WorkerError('UNSUPPORTED_MODEL', 'model_name is not deployed')
                client.timeout = deadline - time.monotonic()
                if handle.is_cancel_requested:
                    raise WorkerError('CANCELED', 'caller canceled before model execution')
                if client.timeout <= 0:
                    raise WorkerError('TIMEOUT', 'request budget exhausted before model execution')
                payload = dict(request_id=request_id, model_name=request.model_name,
                               prompt=request.prompt,
                               system_prompt=request.system_prompt,
                               images=images, timeout=client.timeout)
                output['result'] = client.request('/infer_frames', payload)
            except WorkerError as error:
                output.update(error_code=error.code, message='inference worker request failed')
            except Exception:
                output.update(error_code='INVALID_INPUT', message='worker response invalid')
            finally:
                finished.set()
                # Release only after the blocking call actually exits; no unbounded queue.
                self._release()

        thread = threading.Thread(target=work, daemon=True)
        with self._lock:
            self._threads.add(thread)
        thread.start()
        while not finished.wait(0.05):
            if handle.is_cancel_requested:
                result.error_code, result.message = 'CANCELED', 'caller canceled; execution may remain busy'
                handle.canceled()
                return result
            if time.monotonic() >= deadline or not self.context.ok():
                result.error_code, result.message = 'TIMEOUT', 'request budget exhausted; execution may remain busy'
                handle.abort()
                return result
            handle.publish_feedback(InferVisionLanguageFrames.Feedback(phase='INFERENCING'))
        result.latency_ms = (time.monotonic() - started) * 1000
        if handle.is_cancel_requested:
            result.error_code = 'CANCELED'
            handle.canceled()
            return result
        if time.monotonic() >= deadline:
            result.error_code = 'TIMEOUT'
        elif 'error_code' in output:
            result.error_code, result.message = output['error_code'], output['message']
        else:
            body = output['result']
            echoed = body.get('images') if isinstance(body, dict) else None
            identity_ok = (isinstance(body, dict)
                           and body.get('request_id') == request_id
                           and body.get('model_name') == request.model_name
                           and isinstance(echoed, list) and len(echoed) == len(images)
                           and all(e.get('stamp_ns') == i['stamp_ns']
                                   and e.get('frame') == i['frame']
                                   and e.get('label') == i['label']
                                   for e, i in zip(echoed, images)))
            if not identity_ok:
                result.error_code, result.message = 'BAD_PAYLOAD', 'worker response identity mismatch'
            elif not isinstance(body.get('text'), str) or not isinstance(body.get('error_code'), str):
                result.error_code = 'BAD_PAYLOAD'
            else:
                result.text = body['text']
                result.error_code = body['error_code']
                result.message = body.get('message', '')
        result.success = not result.error_code
        if result.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    def close(self):
        self.server.destroy()
        with self._lock:
            threads = list(self._threads)
        for thread in threads:
            thread.join(timeout=1.0)


def main():
    rclpy.init()
    node = VisionLanguageFramesNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        shutdown_executor(executor)
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
