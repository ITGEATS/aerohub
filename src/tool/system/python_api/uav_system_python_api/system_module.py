from as2_python_api.modules.module_base import ModuleBase
from uav_system_python_api.client import SystemClient


class SystemModule(ModuleBase):
    __alias__ = 'system'

    def __init__(self, drone):
        super().__init__(drone, self.__alias__)
        self.client = SystemClient(drone)

    @property
    def ready(self):
        return self.client.ready

    def __call__(self, operation, **kwargs):
        if operation not in ('takeoff', 'land', 'get_state'):
            raise ValueError('unknown System Tool operation')
        return getattr(self, operation)(**kwargs)

    def takeoff(self, *args, **kwargs):
        return self.client.takeoff(*args, **kwargs)

    def land(self, *args, **kwargs):
        return self.client.land(*args, **kwargs)

    def get_state(self):
        return self.client.get_state()

    def destroy(self):
        self.client.destroy()
