"""Lifecycle client for a Tool that owns native Behavior calls, not trajectories."""
import json
import time

from std_msgs.msg import String
from std_srvs.srv import Trigger


class ToolExecution:
    def __init__(self, node, task_id):
        self.task_id = task_id
        self.prefix = '/planner/tools/task_' + task_id.replace('-', '_')
        self.last_generation = -1
        self.stop_generation_floor = -1
        self.status = None
        self.last_received = None
        self.stop_requested = False
        self.stop_future = None
        self.stop_confirmed = False
        self.stop_started = None
        self.node = node
        self.cancel = node.create_client(Trigger, self.prefix + '/cancel')
        self.resume_client = node.create_client(Trigger, self.prefix + '/resume')
        self.subscription = node.create_subscription(String, self.prefix + '/status', self._status, 10)

    def _status(self, message):
        try:
            status = json.loads(message.data)
            if (not isinstance(status, dict) or status.get('task_id') != self.task_id
                    or status.get('state') not in ('IDLE', 'RUNNING', 'PAUSED', 'STOPPED', 'SUCCEEDED')
                    or status.get('phase') not in ('IDLE', 'READY', 'EXECUTING', 'HOLD', 'STOPPING', 'STOP_FAILED', 'LAND_READY', 'LANDING', 'SUCCEEDED')):
                return
            generation = status.get('generation')
            if type(generation) is not int or generation < self.last_generation:
                return
            self.last_generation = generation
            self.status = status
            self.last_received = time.monotonic()
            if (self.stop_requested and status.get('stop_confirmed') is True
                    and generation > self.stop_generation_floor
                    and self.stop_future is not None and self.stop_future.done()):
                try:
                    self.stop_confirmed = bool(self.stop_future.result().success)
                except Exception:
                    self.stop_confirmed = False
        except (ValueError, TypeError):
            return

    @property
    def quiescent(self):
        return self.stop_requested and self.stop_confirmed

    def stop(self):
        if not self.stop_requested:
            self.stop_started = time.monotonic()
            self.stop_generation_floor = self.last_generation
        self.stop_requested = True
        if self.stop_future is None and self.cancel.service_is_ready():
            self.stop_future = self.cancel.call_async(Trigger.Request())

    def resume(self):
        if not self.quiescent or not self.resume_client.service_is_ready():
            raise ValueError('waiting for Tool stop confirmation/readiness')
        self.stop_requested = self.stop_confirmed = False
        self.stop_future = None
        self.status = None
        return self.resume_client.call_async(Trigger.Request())

    def close(self):
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_client(self.cancel)
        self.node.destroy_client(self.resume_client)
