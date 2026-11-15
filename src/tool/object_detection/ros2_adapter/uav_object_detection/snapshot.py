"""Public single-observation task over the existing RGB-D implementation."""
import math
import threading
import time
import uuid

from sensor_msgs.msg import Image
from rclpy.callback_groups import ReentrantCallbackGroup
from uav_object_detection_interfaces.srv import DetectObjects, DetectSnapshot
from uav_object_detection_interfaces.msg import ObjectObservation


class SnapshotAdapter:
    def __init__(self, node, image_topic):
        self.node = node
        self.gate = threading.Lock()
        self.condition = threading.Condition()
        self.waiting = False
        self.image = None
        self.subscription = node.create_subscription(
            Image, image_topic, self.on_image, node.camera_qos)
        self.server = node.create_service(
            DetectSnapshot, 'tools/object_detection/observe', self.observe,
            callback_group=ReentrantCallbackGroup())

    def on_image(self, image):
        with self.condition:
            if self.waiting and self.image is None:
                self.image = image
                self.condition.notify_all()

    @staticmethod
    def fail(response, code, reason):
        response.success, response.code, response.reason = False, code, reason
        return response

    def observe(self, request, response):
        if not math.isfinite(request.timeout) or request.timeout <= 0:
            return self.fail(response, 'INVALID_REQUEST', 'timeout must be finite and positive')
        if request.category not in ('person', 'car', 'truck', 'bus'):
            return self.fail(response, 'UNSUPPORTED', 'category unsupported by deployed detector')
        if not self.gate.acquire(blocking=False):
            return self.fail(response, 'BUSY', 'Previous observation has not finished')
        deadline = time.monotonic() + request.timeout
        with self.node.lock:
            token = self.node.generation
        with self.condition:
            self.image = None
            self.waiting = True
        done = threading.Event()
        outcome = {}

        def valid():
            return (self.node.active(token) and not self.node.stop_event.is_set()
                    and self.node.context.ok())

        def active(_):
            return valid() and time.monotonic() < deadline

        def work():
            try:
                # Capture only the first frame received after acceptance.
                # The inference layer checks acquisition stamps.
                with self.condition:
                    while self.image is None:
                        if not valid():
                            raise ValueError('STOPPED')
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise TimeoutError('No new RGB frame within task budget')
                        self.condition.wait(min(.02, remaining))
                    image = self.image
                if not active(token):
                    raise TimeoutError('Observation deadline expired')
                task = DetectObjects.Request(request_id=uuid.uuid4().hex,
                                             generation=token, image=image)
                outcome['frame'] = self.node.infer(task, category=request.category,
                                                   observation_active=active,
                                                   estimate_position=request.estimate_position)
            except Exception as error:
                outcome['error'] = error
            finally:
                # Returning TIMEOUT never releases this gate while processing
                # is still running. Late results are not reused by another call.
                with self.condition:
                    self.waiting = False
                    self.image = None
                done.set()
                self.gate.release()

        thread = threading.Thread(target=work, daemon=True)
        thread.start()
        while not done.wait(max(0., min(.02, deadline-time.monotonic()))):
            if not valid():
                return self.fail(response, 'STOPPED', 'Observation invalidated by stop or clock reset')
            if time.monotonic() >= deadline:
                return self.fail(response, 'TIMEOUT', 'Observation budget expired; late result discarded')
        if not valid():
            return self.fail(response, 'STOPPED', 'Observation invalidated by stop or clock reset')
        if time.monotonic() >= deadline:
            return self.fail(response, 'TIMEOUT', 'Observation budget expired; late result discarded')
        error = outcome.get('error')
        if error is not None:
            message = str(error)
            code = ('TIMEOUT' if isinstance(error, TimeoutError) or 'TIMEOUT' in message else
                    'BUSY' if message == 'BUSY' else
                    'STOPPED' if 'STOPPED' in message or 'CANCELLED' in message else
                    'NOT_READY' if message == 'INFERENCE_NOT_READY' else 'DETECTION_FAILED')
            return self.fail(response, code, message)
        frame = outcome['frame']
        response.success = True
        response.code = 'SUCCEEDED' if frame.valid or not frame.detections else 'PARTIAL'
        response.reason = frame.error
        response.result = ObjectObservation(position_requested=request.estimate_position, header=frame.header, image_header=frame.image_header,
            depth_header=frame.depth_header, model_version=frame.model_version, detections=frame.detections)
        return response
