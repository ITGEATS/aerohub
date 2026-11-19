"""Onboard System Tool: managed native takeoff/land; no direct flight setpoints."""
import copy
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from as2_msgs.action import Land, Takeoff
from as2_msgs.msg import PlatformInfo, PlatformStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
from uav_motion_resource.lease import MotionLease
from uav_system_interfaces.action import SystemCommand
from uav_system_interfaces.srv import GetSystemState
from .policy import validate


class SystemTool(Node):
    def __init__(self, **kwargs):
        super().__init__('system_tool', **kwargs)
        self.age = self.declare_parameter('state_max_age', .75).value
        self.settle = self.declare_parameter('settle_seconds', 1.).value
        self.cleanup_timeout = self.declare_parameter('cleanup_timeout', 10.).value
        if not all(math.isfinite(x) and x > 0 for x in (self.age, self.settle, self.cleanup_timeout)):
            raise ValueError('state/settle/cleanup budgets must be finite and positive')
        self.lease = MotionLease(self.get_namespace())
        self.slot = threading.Lock()
        self.data_lock = threading.Lock()
        self.samples = {}
        self.phase = 'idle'
        self.native = None
        self.native_result = None
        self.pending_native = None
        self.pending_service = None
        self.uncertain = False
        self.last_sim = None
        self.last_feedback = 0.
        self.origin_z = 0.
        self.frame = None
        group = ReentrantCallbackGroup()
        for kind, topic, key in ((PlatformInfo,'platform/info','platform'),
                                  (PoseStamped,'self_localization/pose','pose'),
                                  (TwistStamped,'self_localization/twist','twist'),
                                  (Odometry,'sensor_measurements/odom','odom')):
            self.create_subscription(kind, topic, lambda m,k=key:self.receive(k,m), qos_profile_sensor_data)
        self.takeoff_client = ActionClient(self, Takeoff, 'TakeoffBehavior', callback_group=group)
        self.land_client = ActionClient(self, Land, 'LandBehavior', callback_group=group)
        self.arming = self.create_client(SetBool, 'set_arming_state', callback_group=group)
        self.offboard = self.create_client(SetBool, 'set_offboard_mode', callback_group=group)
        self.server = ActionServer(self, SystemCommand, 'tools/system/command',
            execute_callback=self.execute, goal_callback=self.admit,
            cancel_callback=lambda _:CancelResponse.ACCEPT, callback_group=group)
        self.create_service(GetSystemState, 'tools/system/state', self.get_state, callback_group=group)

    def receive(self, key, message):
        with self.data_lock:
            self.samples[key] = (message, time.monotonic())

    def snapshot(self):
        with self.data_lock:
            return dict(self.samples)

    def fresh(self, samples):
        now, wall = self.get_clock().now().nanoseconds/1e9, time.monotonic()
        for key in ('platform','pose','twist'):
            if key not in samples:
                return False
            msg, received = samples[key]
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if not 0 <= wall-received <= self.age or not -.05 <= now-stamp <= self.age:
                return False
        pose, twist = samples['pose'][0], samples['twist'][0]
        p, v = pose.pose.position, twist.twist.linear
        return (bool(pose.header.frame_id) and all(math.isfinite(x) for x in (p.x,p.y,p.z,v.x,v.y,v.z))
                and samples['platform'][0].connected)

    def get_state(self, request, response):
        samples = self.snapshot()
        response.fresh = self.fresh(samples)
        if 'platform' in samples:
            p = samples['platform'][0]
            response.header = p.header
            response.connected, response.armed, response.offboard = p.connected, p.armed, p.offboard
            response.platform_state = p.status.state
        if 'pose' in samples:
            response.pose = samples['pose'][0]
        if 'twist' in samples:
            response.twist = samples['twist'][0]
        if 'odom' in samples:
            odom, received = samples['odom']
            response.navigation_pose.header = copy.deepcopy(odom.header)
            response.navigation_pose.pose = copy.deepcopy(odom.pose.pose)
            stamp = odom.header.stamp.sec+odom.header.stamp.nanosec/1e9
            now = self.get_clock().now().nanoseconds/1e9
            p = odom.pose.pose.position
            response.navigation_pose_fresh = bool(odom.header.frame_id and
                0 <= time.monotonic()-received <= self.age and -.05 <= now-stamp <= self.age and
                all(math.isfinite(x) for x in (p.x,p.y,p.z)))
        response.motion_owner = self.lease.owner()
        response.busy, response.phase = bool(response.motion_owner), self.phase
        return response

    def admit(self, request):
        if validate(request.command, request.height, request.speed, request.timeout):
            return GoalResponse.REJECT
        if not self.slot.acquire(blocking=False):
            return GoalResponse.REJECT
        try:
            if not self.lease.acquire('system.takeoff' if request.command==1 else 'system.land'):
                self.slot.release()
                return GoalResponse.REJECT
        except Exception:
            self.slot.release()
            raise
        return GoalResponse.ACCEPT

    def check(self, handle, deadline, cancel=True):
        if time.monotonic() >= deadline:
            raise TimeoutError('operation wall-clock deadline exceeded')
        if cancel and handle.is_cancel_requested:
            raise InterruptedError('system command canceled')
        now = self.get_clock().now().nanoseconds
        if self.last_sim is not None and now < self.last_sim:
            raise RuntimeError('simulation clock regressed')
        self.last_sim = now
        samples = self.snapshot()
        if self.frame and 'pose' in samples and samples['pose'][0].header.frame_id != self.frame:
            raise RuntimeError('self-localization frame changed during command')
        if time.monotonic()-self.last_feedback > .2:
            samples = self.snapshot()
            if 'platform' in samples and 'pose' in samples:
                handle.publish_feedback(SystemCommand.Feedback(phase=self.phase,
                    relative_height=samples['pose'][0].pose.position.z-self.origin_z,
                    armed=samples['platform'][0].armed))
                self.last_feedback = time.monotonic()

    def wait(self, predicate, handle, deadline, require_state=True, cancel=True):
        while True:
            self.check(handle, deadline, cancel)
            if require_state and not self.fresh(self.snapshot()):
                raise RuntimeError('platform/pose/twist missing, stale or invalid')
            if predicate():
                return
            time.sleep(.02)

    def service(self, client, value, handle, deadline, cancel=True):
        self.wait(client.service_is_ready, handle, deadline, cancel=cancel)
        self.uncertain = True
        future = client.call_async(SetBool.Request(data=value))
        self.pending_service = future
        # The locked AS2 platform handles mode/arming services synchronously;
        # its state publisher pauses while waiting for PX4 command ACK. Settle
        # the atomic service outcome, then reacquire fresh inputs before motion.
        self.wait(future.done, handle, deadline, require_state=False, cancel=cancel)
        response = future.result()
        self.uncertain = False
        self.pending_service = None
        if response is None or not response.success:
            raise RuntimeError('platform service rejected request')
        self.wait(lambda:self.fresh(self.snapshot()), handle, deadline,
                  require_state=False, cancel=cancel)

    def stopped(self):
        samples = self.snapshot()
        if not self.fresh(samples):
            return False
        v = samples['twist'][0].twist.linear
        return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z) < .25

    def stable(self, handle, deadline, predicate=lambda:True, cancel=True):
        since = None
        while True:
            self.check(handle, deadline, cancel)
            if self.stopped() and predicate():
                since = since or time.monotonic()
                if time.monotonic()-since >= self.settle:
                    return
            else:
                since = None
            time.sleep(.02)

    def submit_native(self, client, goal, handle, deadline):
        self.wait(client.server_is_ready, handle, deadline)
        self.uncertain = True
        future = client.send_goal_async(goal)
        self.pending_native = future
        self.wait(future.done, handle, deadline)
        self.native = future.result()
        self.uncertain = False
        if self.native is None or not self.native.accepted:
            self.native = None
            raise RuntimeError('native Behavior rejected goal')
        self.native_result = self.native.get_result_async()
        self.wait(self.native_result.done, handle, deadline)
        result = self.native_result.result()
        ok = result.result.takeoff_success if isinstance(goal, Takeoff.Goal) else result.result.land_success
        if result.status != GoalStatus.STATUS_SUCCEEDED or not ok:
            raise RuntimeError('native Behavior failed')

    def cleanup(self, handle):
        deadline = time.monotonic()+self.cleanup_timeout
        if self.uncertain:
            if self.pending_native is None:
                if self.pending_service is None:
                    return False
                self.wait(self.pending_service.done, handle, deadline, require_state=False, cancel=False)
                # Do not hand off until the service outcome is known and fresh
                # platform inputs confirm its physical effect below.
                self.pending_service.result()
                self.pending_service = None
                self.uncertain = False
                self.wait(lambda:self.fresh(self.snapshot()), handle, deadline,
                          require_state=False, cancel=False)
            try:
                if self.pending_native is not None:
                    self.wait(self.pending_native.done, handle, deadline, require_state=False, cancel=False)
                self.native = self.pending_native.result() if self.pending_native is not None else None
                if self.native is not None and self.native.accepted:
                    self.native_result = self.native.get_result_async()
                else:
                    self.native = None
                self.uncertain = False
            except Exception:
                def cancel_late(done):
                    native = done.result()
                    if native and native.accepted:
                        native.cancel_goal_async()
                if self.pending_native is not None:
                    self.pending_native.add_done_callback(cancel_late)
                return False
        if self.native is not None and self.native_result is not None and not self.native_result.done():
            future = self.native.cancel_goal_async()
            self.wait(future.done, handle, deadline, require_state=False, cancel=False)
            self.wait(self.native_result.done, handle, deadline, require_state=False, cancel=False)
        samples = self.snapshot()
        if (self.fresh(samples) and samples['platform'][0].armed and
                samples['platform'][0].status.state in (PlatformStatus.LANDED, PlatformStatus.DISARMED)):
            self.service(self.arming, False, handle, deadline, cancel=False)
            self.wait(lambda:not self.snapshot()['platform'][0].armed, handle, deadline, cancel=False)
        self.stable(handle, deadline, cancel=False)
        return True

    def execute(self, handle):
        request = handle.request
        deadline = time.monotonic()+request.timeout
        self.native = self.native_result = self.pending_native = self.pending_service = None
        self.uncertain = False
        self.last_sim = None
        self.frame = None
        success = stopped = False
        code, reason = 'FAILED', ''
        try:
            self.phase = 'preparing'
            self.wait(lambda:self.fresh(self.snapshot()), handle, deadline, require_state=False)
            samples = self.snapshot()
            platform = samples['platform'][0]
            self.origin_z = samples['pose'][0].pose.position.z
            self.frame = samples['pose'][0].header.frame_id
            if request.command == SystemCommand.Goal.TAKEOFF:
                if platform.armed or platform.status.state not in (PlatformStatus.DISARMED, PlatformStatus.LANDED):
                    raise RuntimeError('takeoff requires a disarmed landed platform')
                # Check dependencies before any arming side effect.
                self.wait(self.takeoff_client.server_is_ready, handle, deadline)
                self.wait(self.offboard.service_is_ready, handle, deadline)
                self.service(self.arming, True, handle, deadline)
                self.wait(lambda:self.snapshot()['platform'][0].armed, handle, deadline)
                self.service(self.offboard, True, handle, deadline)
                self.wait(lambda:self.snapshot()['platform'][0].offboard, handle, deadline)
                self.phase = 'taking_off'
                self.submit_native(self.takeoff_client, Takeoff.Goal(takeoff_height=request.height,
                                   takeoff_speed=request.speed), handle, deadline)
                self.phase = 'settling'
                self.stable(handle, deadline, lambda: self.snapshot()['platform'][0].status.state==PlatformStatus.FLYING
                    and abs(self.snapshot()['pose'][0].pose.position.z-self.origin_z-request.height)<=.5)
            else:
                if platform.status.state in (PlatformStatus.DISARMED, PlatformStatus.LANDED):
                    self.phase = 'landed'
                elif platform.armed and platform.offboard and platform.status.state==PlatformStatus.FLYING:
                    self.phase = 'landing'
                    self.submit_native(self.land_client, Land.Goal(land_speed=request.speed), handle, deadline)
                else:
                    raise RuntimeError('land requires flying/offboard or already landed state')
                self.wait(lambda:self.snapshot()['platform'][0].status.state in
                          (PlatformStatus.LANDED, PlatformStatus.DISARMED), handle, deadline)
                if self.snapshot()['platform'][0].armed:
                    self.service(self.arming, False, handle, deadline)
                self.stable(handle, deadline, lambda:not self.snapshot()['platform'][0].armed)
            stopped, success, code, reason = True, True, 'SUCCEEDED', 'physical completion confirmed'
        except Exception as exc:
            code = 'CANCELED' if isinstance(exc, InterruptedError) else ('TIMEOUT' if isinstance(exc, TimeoutError) else 'FAILED')
            reason = str(exc)
            self.phase = 'stopping'
            try:
                stopped = self.cleanup(handle)
            except Exception as cleanup_error:
                reason += '; cleanup: '+str(cleanup_error)
        finally:
            self.phase = 'idle' if stopped else 'blocked'
            self.lease.release(stopped)
            self.slot.release()
        samples = self.snapshot()
        p = samples.get('platform', (None,))[0]
        result = SystemCommand.Result(success=success, code=code, reason=reason, stop_confirmed=stopped,
            landed=bool(p and self.fresh(samples) and p.status.state in (PlatformStatus.LANDED,PlatformStatus.DISARMED)),
            disarmed=bool(p and self.fresh(samples) and not p.armed))
        if success:
            handle.succeed()
        elif handle.is_cancel_requested and code=='CANCELED':
            handle.canceled()
        else:
            handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SystemTool()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        # Unconfirmed process termination must retain the motion journal.
        node.lease.release(False)
        executor.shutdown(timeout_sec=5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
