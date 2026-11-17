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

    def __call__(self, query='car', timeout=10.0, feedback_callback=None, *, follow=False):
        return self.track(query, timeout, feedback_callback, follow=follow)

    def track(self, query='car', timeout=10.0, feedback_callback=None, *, follow=False):
        if not isinstance(query, str) or not query.strip() or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('query and finite positive timeout required')
        if not isinstance(follow, bool):
            raise ValueError('follow must be bool')
        if self._client is None:
            raise RuntimeError('Tracking module destroyed')
        return self._client.send_goal_async(TrackObject.Goal(query=query, timeout=float(timeout), follow=follow), feedback_callback=feedback_callback)

    def destroy(self):
        if self._client is not None:
            self._client.destroy()
            self._client = None
