"""Real generated ROS transport with a controllable platform/Behavior peer.

Synthetic lifecycle evidence only; not a flight acceptance substitute.
"""
import os
import tempfile
import threading
import time
import unittest

import rclpy
from as2_msgs.action import Land, Takeoff
from as2_msgs.msg import PlatformInfo, PlatformStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
from uav_motion_resource.lease import MotionLease
from uav_system_python_api.client import SystemClient
from uav_system_tool.node import SystemTool


def wait_for(predicate, timeout=8):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>deadline:
            raise TimeoutError('test wait deadline')
        time.sleep(.01)


class Platform(Node):
    def __init__(self, namespace):
        super().__init__('platform_peer',namespace=namespace)
        self.armed = self.offboard = False
        self.state = PlatformStatus.DISARMED
        self.height = 0.
        self.delay = 0.
        self.mode_delay = 0.
        self.fail = False
        self.publishing = True
        self.calls = []
        group = ReentrantCallbackGroup()
        self.platform = self.create_publisher(PlatformInfo,'platform/info',qos_profile_sensor_data)
        self.pose = self.create_publisher(PoseStamped,'self_localization/pose',qos_profile_sensor_data)
        self.twist = self.create_publisher(TwistStamped,'self_localization/twist',qos_profile_sensor_data)
        self.create_timer(.02,self.publish)
        self.create_service(SetBool,'set_arming_state',self.arm,callback_group=group)
        self.create_service(SetBool,'set_offboard_mode',self.mode,callback_group=group)
        self.takeoff_server = ActionServer(self,Takeoff,'TakeoffBehavior',self.takeoff,
            callback_group=group, cancel_callback=self.cancel)
        self.land_server = ActionServer(self,Land,'LandBehavior',self.land,callback_group=group)

    def publish(self):
        if not self.publishing:
            return
        stamp = self.get_clock().now().to_msg()
        platform = PlatformInfo(connected=True,armed=self.armed,offboard=self.offboard)
        platform.header.stamp = stamp
        platform.status.state = self.state
        pose, twist = PoseStamped(), TwistStamped()
        for msg in (pose,twist):
            msg.header.stamp, msg.header.frame_id = stamp, 'earth'
        pose.pose.position.z = self.height
        pose.pose.orientation.w = 1.
        self.platform.publish(platform)
        self.pose.publish(pose)
        self.twist.publish(twist)

    def arm(self, request, response):
        self.calls.append(('arm',request.data))
        self.armed = request.data
        self.state = PlatformStatus.LANDED if request.data else PlatformStatus.DISARMED
        response.success = True
        return response

    def mode(self, request, response):
        self.calls.append(('offboard',request.data))
        if self.mode_delay:
            self.publishing = False
            time.sleep(self.mode_delay)
            self.publishing = True
        self.offboard = request.data
        response.success = True
        return response

    def cancel(self, request):
        from rclpy.action import CancelResponse
        return CancelResponse.ACCEPT

    def takeoff(self, handle):
        self.calls.append(('takeoff',handle.request.takeoff_height))
        until = time.monotonic()+self.delay
        while time.monotonic()<until:
            if handle.is_cancel_requested:
                handle.canceled()
                return Takeoff.Result(takeoff_success=False)
            time.sleep(.01)
        if self.fail:
            handle.abort()
            return Takeoff.Result(takeoff_success=False)
        self.height = float(handle.request.takeoff_height)
        self.state = PlatformStatus.FLYING
        handle.succeed()
        return Takeoff.Result(takeoff_success=True)

    def land(self, handle):
        self.calls.append(('land',handle.request.land_speed))
        self.height, self.state, self.armed = 0., PlatformStatus.LANDED, False
        handle.succeed()
        return Land.Result(land_success=True)


class RosTests(unittest.TestCase):
    counter = 0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.directory = tempfile.TemporaryDirectory()
        os.environ['AEROHUB_MOTION_LOCK_DIR'] = cls.directory.name

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        cls.directory.cleanup()

    def setUp(self):
        type(self).counter += 1
        self.ns = '/system_test'+str(self.counter)
        self.tool = SystemTool(namespace=self.ns, parameter_overrides=[
            Parameter('settle_seconds',value=.1),Parameter('cleanup_timeout',value=.5),
            Parameter('state_max_age',value=.2)])
        self.peer = Platform(self.ns)
        self.node = Node('test_client',namespace=self.ns)
        self.client = SystemClient(self.node)
        self.executor = MultiThreadedExecutor(num_threads=8)
        for node in (self.tool,self.peer,self.node):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin,daemon=True)
        self.thread.start()
        wait_for(lambda:self.client.ready and self.tool.fresh(self.tool.snapshot()))

    def tearDown(self):
        self.executor.shutdown(timeout_sec=5)
        self.thread.join(5)
        self.client.destroy()
        for node in (self.tool,self.peer,self.node):
            node.destroy_node()

    def result(self, future):
        wait_for(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        wait_for(result.done)
        return result.result().result

    def test_takeoff_land_and_query(self):
        result = self.result(self.client.takeoff())
        self.assertTrue(result.success and result.stop_confirmed)
        self.assertFalse(result.disarmed)
        state = self.client.get_state()
        wait_for(state.done)
        self.assertTrue(state.result().fresh)
        self.assertEqual(state.result().pose.header.frame_id, "earth")
        self.assertAlmostEqual(state.result().pose.pose.position.z, 3.)
        self.assertAlmostEqual(state.result().twist.twist.linear.z, 0.)
        result = self.result(self.client.land())
        self.assertTrue(result.success and result.stop_confirmed and result.landed and result.disarmed)
        state = self.client.get_state()
        wait_for(state.done)
        self.assertTrue(state.result().fresh)
        self.assertFalse(state.result().busy)
        self.assertEqual([x[0] for x in self.peer.calls],['arm','offboard','takeoff','land'])

    def test_platform_service_pause_reacquires_fresh_state_before_takeoff(self):
        self.peer.mode_delay = 1.
        result = self.result(self.client.takeoff())
        self.assertTrue(result.success and result.stop_confirmed, result.reason)
        self.assertIn(('takeoff', 3.), self.peer.calls)
        self.result(self.client.land())

    def test_cancel_during_platform_service_settles_then_rolls_back(self):
        self.peer.mode_delay = .25
        submission = self.client.takeoff()
        wait_for(submission.done)
        handle = submission.result()
        wait_for(lambda:('offboard', True) in self.peer.calls)
        cancel = handle.cancel_goal_async()
        wait_for(cancel.done)
        result = handle.get_result_async()
        wait_for(result.done)
        self.assertTrue(result.result().result.stop_confirmed, result.result().result.reason)
        self.assertFalse(self.peer.armed)
        self.assertFalse(any(name=='takeoff' for name,_ in self.peer.calls))

    def test_shared_navigation_ownership_rejects_system(self):
        nav = MotionLease(self.ns)
        self.assertTrue(nav.acquire('navigation.super'))
        try:
            future = self.client.takeoff()
            wait_for(future.done)
            self.assertFalse(future.result().accepted)
            self.assertEqual(self.peer.calls,[])
        finally:
            nav.release(True)

    def test_system_rejects_competing_navigation_and_system(self):
        self.peer.delay = .5
        future = self.client.takeoff()
        wait_for(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        self.assertFalse(MotionLease(self.ns).acquire('navigation.super'))
        duplicate = self.client.land()
        wait_for(duplicate.done)
        self.assertFalse(duplicate.result().accepted)
        result = handle.get_result_async()
        wait_for(result.done)
        self.assertTrue(result.result().result.success)
        self.result(self.client.land())

    def test_cancel_drains_native_and_rolls_back_ground_arming(self):
        self.peer.delay = 3.
        future = self.client.takeoff()
        wait_for(future.done)
        handle = future.result()
        wait_for(lambda:any(name=='takeoff' for name,_ in self.peer.calls))
        canceled = handle.cancel_goal_async()
        wait_for(canceled.done)
        result = handle.get_result_async()
        wait_for(result.done)
        self.assertEqual(result.result().result.code,'CANCELED')
        self.assertTrue(result.result().result.stop_confirmed, result.result().result.reason)
        self.assertFalse(self.peer.armed)
        self.assertFalse(self.tool.lease.owner())

    def test_native_failure_does_not_report_success(self):
        self.peer.fail = True
        result = self.result(self.client.takeoff())
        self.assertFalse(result.success)
        self.assertTrue(result.stop_confirmed and result.disarmed)

    def test_invalid_request_has_no_platform_side_effects(self):
        future = self.client.takeoff(height=-1.)
        wait_for(future.done)
        self.assertFalse(future.result().accepted)
        self.assertEqual(self.peer.calls,[])

    def test_stale_input_retains_ownership(self):
        self.peer.publishing = False
        time.sleep(.3)
        result = self.result(self.client.takeoff(timeout=5.))
        self.assertFalse(result.success or result.stop_confirmed)
        future = self.client.land()
        wait_for(future.done)
        self.assertFalse(future.result().accepted)
        self.assertEqual(self.peer.calls,[])


if __name__=='__main__':
    unittest.main()
