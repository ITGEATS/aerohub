"""Task-facing ROS Tool client. Futures require a separately spinning executor."""
from rclpy.action import ActionClient
from uav_system_interfaces.action import SystemCommand
from uav_system_interfaces.srv import GetSystemState


class SystemClient:
    def __init__(self, node):
        self.action = ActionClient(node, SystemCommand, 'tools/system/command')
        self.state = node.create_client(GetSystemState, 'tools/system/state')

    @property
    def ready(self):
        return self.action.server_is_ready() and self.state.service_is_ready()

    def takeoff(self, height=3., speed=.5, timeout=60., feedback_callback=None):
        return self.action.send_goal_async(SystemCommand.Goal(command=SystemCommand.Goal.TAKEOFF,
            height=float(height), speed=float(speed), timeout=float(timeout)), feedback_callback=feedback_callback)

    def land(self, speed=.5, timeout=90., feedback_callback=None):
        return self.action.send_goal_async(SystemCommand.Goal(command=SystemCommand.Goal.LAND,
            speed=float(speed), timeout=float(timeout)), feedback_callback=feedback_callback)

    def get_state(self):
        return self.state.call_async(GetSystemState.Request())

    def destroy(self):
        self.action.destroy()
        self.state.destroy()
