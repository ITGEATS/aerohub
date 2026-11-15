"""AS2 application entry; DroneInterface's executor must be spinning separately."""
import math
import threading
import time

from as2_python_api.modules.module_base import ModuleBase
from uav_object_detection_interfaces.srv import DetectSnapshot


class ObjectDetectionModule(ModuleBase):
    __alias__ = 'object_detection'

    def __init__(self, drone):
        super().__init__(drone, self.__alias__)
        self._drone = drone
        self._client = drone.create_client(DetectSnapshot, 'tools/object_detection/observe')
        self._lock = threading.Lock()

    @property
    def ready(self):
        """Service discovery only; sensor/model readiness is checked per call."""
        return self._client.service_is_ready()

    def __call__(self, category='person', timeout=2.0, *, estimate_position=False):
        return self.detect(category, timeout, estimate_position=estimate_position)

    def detect(self, category='person', timeout=2.0, *, estimate_position=False):
        if not isinstance(estimate_position, bool):
            raise ValueError('estimate_position must be a bool')
        if not isinstance(category, str) or not category:
            raise ValueError('category must be a nonempty string')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        if not self._lock.acquire(blocking=False):
            return DetectSnapshot.Response(success=False, code='BUSY', reason='Client observation already running')
        try:
            deadline = time.monotonic() + timeout
            if not self._client.wait_for_service(timeout_sec=timeout):
                return DetectSnapshot.Response(success=False, code='NOT_READY', reason='Detection service unavailable')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return DetectSnapshot.Response(success=False, code='TIMEOUT', reason='Discovery exhausted task budget')
            future = self._client.call_async(DetectSnapshot.Request(category=category, timeout=float(remaining), estimate_position=estimate_position))
            # The server enforces the observation budget. Allow a small transport
            # margin, bounded on the client wall clock even if the server disappears.
            deadline += .5
            while not future.done():
                if not self._drone.context.ok():
                    raise RuntimeError('ROS context stopped')
                if time.monotonic() >= deadline:
                    future.cancel()
                    return DetectSnapshot.Response(success=False, code='TIMEOUT', reason='No response; late result discarded')
                time.sleep(.005)
            return future.result()
        finally:
            self._lock.release()

    def destroy(self):
        self._drone.destroy_client(self._client)
