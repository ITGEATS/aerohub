"""AS2 module; the DroneInterface executor must be spinning separately."""
import math
from as2_python_api.modules.module_base import ModuleBase
from rclpy.action import ActionClient
from uav_target_tracking_interfaces.action import TrackObject


class TargetTrackingModule(ModuleBase):
    __alias__ = 'target_tracking'

    def __init__(self, drone):
        super().__init__(drone, self.__alias__)
        self._client = ActionClient(drone, TrackObject, 'tools/target_tracking/track')

    @property
    def ready(self):
        return self._client is not None and self._client.server_is_ready()

    def __call__(self, category='person', timeout=10.0, feedback_callback=None):
        return self.track(category, timeout, feedback_callback)

    def track(self, category='person', timeout=10.0, feedback_callback=None):
        if not isinstance(category, str) or not category.strip() or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('category and finite positive timeout required')
        if self._client is None:
            raise RuntimeError('Tracking module destroyed')
        return self._client.send_goal_async(TrackObject.Goal(category=category, timeout=float(timeout)), feedback_callback=feedback_callback)

    def destroy(self):
        if self._client is not None:
            self._client.destroy()
            self._client = None
