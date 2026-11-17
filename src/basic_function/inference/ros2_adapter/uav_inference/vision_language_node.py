"""Generic image/text inference Function, with private model-name worker routing.

The configured RGB topic is vehicle-local. No SPF prompts, point/depth schema or
motion logic is present. One cloud request remains busy until the HTTP call ends,
even after the ROS caller cancels or times out.
"""
import base64
import math
import threading
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from uav_inference_interfaces.action import InferVisionLanguage

from .model_client import ModelClient, WorkerError


def jpeg_image(msg):
    if msg.encoding not in ('rgb8', 'bgr8') or not 0 < msg.width <= 4096 or not 0 < msg.height <= 4096 \
            or msg.step < msg.width * 3 or len(msg.data) != msg.step * msg.height:
        raise ValueError('expected bounded rgb8/bgr8 image with valid row stride')
    image = np.ndarray((msg.height, msg.width, 3), dtype=np.uint8,
                       buffer=bytes(msg.data), strides=(msg.step, 3, 1))
    if msg.encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode('.jpg', image)
    if not ok:
        raise ValueError('image encoding failed')
    return base64.b64encode(encoded.tobytes()).decode()


class VisionLanguageNode(Node):
    def __init__(self):
        super().__init__('vision_language_inference')
        self.endpoint = self.declare_parameter('endpoint', 'http://127.0.0.1:8013').value
        camera = self.declare_parameter('camera_topic', 'sensor_measurements/FrontCamera/rgb/image').value
        self._lock = threading.Lock()
        self._busy = False
        self._frame = None
        self._threads = set()
        self.create_subscription(Image, camera, self._image, qos_profile_sensor_data)
        self.server = ActionServer(
            self, InferVisionLanguage, 'inference/vision_language/infer',
            goal_callback=self._goal, cancel_callback=lambda handle: CancelResponse.ACCEPT,
            execute_callback=self._execute, callback_group=ReentrantCallbackGroup())

    def _image(self, msg):
        with self._lock:
            self._frame = (msg, time.monotonic())

    def _goal(self, request):
        if not request.model_name.strip() or not request.prompt.strip() \
                or len(request.model_name) > 128 or len(request.prompt) > 65536 \
                or not all(math.isfinite(v) and v > 0 for v in (request.timeout, request.max_result_age)):
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, handle):
        request = handle.request
        result = InferVisionLanguage.Result(model_name=request.model_name)
        started = time.monotonic()
        deadline = started + request.timeout
        with self._lock:
            frame = self._frame
        if frame is None:
            with self._lock:
                self._busy = False
            result.error_code, result.message = 'NOT_READY', 'no camera image'
            handle.abort()
            return result
        msg, received = frame
        result.observation_header = msg.header
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if not msg.header.frame_id or age < -0.05 or age > request.max_result_age \
                or time.monotonic() - received > request.max_result_age:
            with self._lock:
                self._busy = False
            result.error_code, result.message = 'STALE_INPUT', 'camera image is stale/future or has no frame'
            handle.abort()
            return result
        finished = threading.Event()
        output = {}
        request_id = uuid.uuid4().hex

        def work():
            try:
                if handle.is_cancel_requested:
                    raise WorkerError('CANCELED', 'caller canceled before inference')
                encoded = jpeg_image(msg)
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
                               prompt=request.prompt, image_base64=encoded, stamp_ns=stamp_ns,
                               frame=msg.header.frame_id, timeout=client.timeout)
                output['result'] = client.request('/infer', payload)
            except WorkerError as error:
                output.update(error_code=error.code, message='inference worker request failed')
            except Exception:
                output.update(error_code='INVALID_INPUT', message='image or worker response invalid')
            finally:
                finished.set()
                # Release only after the blocking call actually exits; no unbounded queue.
                with self._lock:
                    self._busy = False
                    self._threads.discard(threading.current_thread())

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
            handle.publish_feedback(InferVisionLanguage.Feedback(phase='INFERENCING'))
        result.latency_ms = (time.monotonic() - started) * 1000
        if handle.is_cancel_requested:
            result.error_code = 'CANCELED'
            handle.canceled()
            return result
        if time.monotonic() >= deadline:
            result.error_code = 'TIMEOUT'
        elif max(age + time.monotonic() - started, time.monotonic() - received,
                 (self.get_clock().now().nanoseconds - stamp_ns) / 1e9) > request.max_result_age:
            result.error_code = 'STALE_RESULT'
        elif 'error_code' in output:
            result.error_code, result.message = output['error_code'], output['message']
        else:
            body = output['result']
            if not isinstance(body, dict) or body.get('request_id') != request_id or body.get('model_name') != request.model_name \
                    or body.get('stamp_ns') != stamp_ns or body.get('frame') != msg.header.frame_id:
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
    node = VisionLanguageNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
