"""Actual ROS Action + AS2 Module tests with a controlled execution double.

This verifies the wrapper protocol, not SUPER planning or flight performance.
"""
import copy
import math
import threading
import time
import unittest
from unittest.mock import patch

import rclpy
from action_msgs.msg import GoalStatus
from as2_python_api.drone_interface_base import DroneInterfaceBase
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from uav_planner_interfaces.action import NavigateToPose
from uav_super_adapter.node import SuperAdapter


class ExecutionDouble:
    def __init__(self, *args):
        self.revision = 0
        self.complete = False
        self.pending = set()
        self.ready = True
        self.cancel_started = False
        self.quiescent = False
        self.settling_vehicles = set()
    def tick(self, now):
        pass
    def close(self):
        self.cancel_started = True
        self.quiescent = True


class NavigationInterfaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.patch = patch('uav_super_adapter.node.FleetFollowPathClient', ExecutionDouble)
        cls.patch.start()
        cls.adapter = SuperAdapter()
        cls.adapter.set_parameters([Parameter('use_sim_time', value=False)])
        # Construct the wrapper after configuring the normal node, as the launch
        # parameter does in production; no source planner process is involved.
        from uav_super_adapter.navigation import SuperNavigation
        cls.adapter.navigation = SuperNavigation(cls.adapter)
        cls.inputs = Node('navigation_test_inputs')
        cls.pose_pub = cls.inputs.create_publisher(Odometry, 'sensor_measurements/odom', qos_profile_sensor_data)
        cls.goals = []
        cls.goal_sub = cls.inputs.create_subscription(PoseStamped, '/goal_pose', cls.goals.append, 1)
        cls.timer = cls.inputs.create_timer(.02, cls.publish_pose)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.adapter)
        cls.executor.add_node(cls.inputs)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        cls.drone = DroneInterfaceBase('drone0')
        try:
            cls.drone.load_module('navigation')
        except Exception:
            cls.drone.shutdown()
            cls.executor.shutdown()
            cls.thread.join()
            raise
        cls.nav = cls.drone.navigation
        deadline = time.monotonic() + 5
        while (not cls.nav.ready or cls.adapter.navigation.odom is None or
               cls.adapter.navigation.publisher.get_subscription_count() == 0):
            if time.monotonic() > deadline:
                raise RuntimeError('ROS discovery timeout')
            time.sleep(.02)

    @classmethod
    def publish_pose(cls):
        msg = Odometry()
        msg.header.frame_id = 'drone0/odom'
        msg.header.stamp = cls.inputs.get_clock().now().to_msg()
        msg.pose.pose.position.z = 2.
        msg.pose.pose.orientation.w = 1.
        cls.pose_pub.publish(msg)

    @classmethod
    def tearDownClass(cls):
        cls.drone.shutdown()
        cls.drone.destroy_node()
        cls.executor.shutdown()
        cls.thread.join()
        cls.inputs.destroy_node()
        cls.adapter.navigation.server.destroy()
        cls.adapter.destroy_node()
        cls.patch.stop()
        rclpy.shutdown()

    def setUp(self):
        self.assertFalse(self.adapter.navigation.reserved)
        self.adapter.error = ''
        self.adapter.enabled = False
        self.adapter.client = None
        self.goals.clear()

    def wait_result(self):
        deadline = time.monotonic() + 5
        while not self.nav.done:
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.02)
        return self.nav.result

    def test_invalid_frame_has_reason_and_no_execution(self):
        target = PoseStamped()
        target.header.frame_id = 'wrong'
        self.nav(target, wait=False)
        self.assertEqual(self.wait_result().code, 'INVALID_GOAL')
        self.assertEqual(self.nav.result_status, GoalStatus.STATUS_ABORTED)
        self.assertIsNone(self.adapter.client)
        self.assertEqual(self.goals, [])

    def test_pose_orientation_is_not_silently_dropped(self):
        target = PoseStamped()
        target.header.frame_id = 'drone0/odom'
        target.pose.orientation.w = 1.
        self.nav.to_pose(target, wait=False)
        self.assertEqual(self.wait_result().code, 'UNSUPPORTED')
        self.assertIsNone(self.adapter.client)

    def test_nonfinite_and_unsupported_speed(self):
        for speed, code in [(math.nan, 'INVALID_GOAL'), (4., 'UNSUPPORTED')]:
            self.nav.to_point(0, 0, 2, max_speed=speed, wait=False)
            self.assertEqual(self.wait_result().code, code)
            self.assertIsNone(self.adapter.client)

    def test_point_completion_and_repeated_task(self):
        for _ in range(2):
            self.nav.to_point(0, 0, 2, wait=False, timeout=5.)
            deadline = time.monotonic() + 2
            while not self.adapter.client:
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.01)
            self.adapter.client.revision = 1
            self.adapter.client.complete = True
            result = self.wait_result()
            self.assertTrue(result.success, result.reason)
            self.assertTrue(result.stop_confirmed)
            self.assertEqual(self.nav.result_status, GoalStatus.STATUS_SUCCEEDED)
            self.assertIsNotNone(self.nav.feedback)
        self.assertEqual(len(self.goals), 2)
        self.assertEqual(self.goals[-1].pose.orientation.w, 1.)

    def test_cancel_and_legacy_service_cannot_bypass_task(self):
        from std_srvs.srv import SetBool
        self.nav.to_point(0, 4, 2, wait=False)
        deadline = time.monotonic() + 2
        while not self.adapter.enabled:
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.01)
        response = self.adapter.enable(SetBool.Request(data=False), SetBool.Response())
        self.assertFalse(response.success)
        self.assertTrue(self.adapter.enabled)
        acknowledged=self.nav._wait(self.nav.cancel_async(), 2.)
        self.assertTrue(acknowledged.goals_canceling)
        self.wait_result()
        self.assertTrue(self.nav.cancel())
        self.assertEqual(self.nav.result.code, 'CANCELED')
        self.assertEqual(self.nav.result_status, GoalStatus.STATUS_CANCELED)
        self.assertTrue(self.adapter.client.quiescent)

    def test_busy_goal_and_execution_failure(self):
        from rclpy.action import ActionClient
        self.nav.to_point(0, 4, 2, wait=False)
        other = ActionClient(self.drone, NavigateToPose, 'navigate_to_pose')
        try:
            request = NavigateToPose.Goal()
            request.target.header.frame_id = 'drone0/odom'
            response = self.nav._wait(other.send_goal_async(request), 2.)
            self.assertFalse(response.accepted)
            self.adapter.fail('controlled source failure')
            result = self.wait_result()
            self.assertEqual(result.code, 'EXECUTION_FAILED')
            self.assertIn('controlled source failure', result.reason)
            self.assertTrue(result.stop_confirmed)
        finally:
            other.destroy()

    def wait_for(self, predicate, timeout=5.):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.01)

    def test_running_target_update_keeps_execution(self):
        self.nav.to_point(0, 8, 2, wait=False)
        self.wait_for(lambda: self.adapter.enabled)
        task, client, handle = self.adapter.task, self.adapter.client, self.nav._handle
        self.nav.to_point(0, 12, 2, wait=False)
        self.assertEqual(self.adapter.goal, [0., 12., 2.])
        self.assertEqual(self.adapter.task, task)
        self.assertIs(self.adapter.client, client)
        self.assertIs(self.nav._handle, handle)
        from types import SimpleNamespace
        old_feedback = self.nav.feedback
        self.nav._on_feedback(SimpleNamespace(goal_id=None, feedback='old-session'))
        self.assertIs(self.nav.feedback, old_feedback)
        self.assertFalse(client.cancel_started)
        self.wait_for(lambda: self.nav.feedback and self.nav.feedback.target_revision == 2)
        self.assertAlmostEqual(self.nav.feedback.distance_remaining, 12.)
        self.nav.cancel()

    def test_update_identity_revision_and_validation(self):
        from uav_planner_interfaces.srv import UpdateNavigationTarget
        self.nav.to_point(0, 8, 2, wait=False)
        self.wait_for(lambda: self.adapter.enabled)
        request = UpdateNavigationTarget.Request(target_revision=2)
        request.target.header.frame_id = 'drone0/odom'
        def update():
            return self.nav._wait(self.nav._updates.call_async(request), 2.)
        self.assertEqual(update().code, 'NOT_OWNER')
        request.goal_id = self.nav._handle.goal_id
        request.target_revision = 1
        self.assertEqual(update().code, 'STALE_TARGET')
        request.target_revision = 2
        request.target.header.frame_id = 'wrong'
        self.assertEqual(update().code, 'INVALID_GOAL')
        self.assertEqual(self.adapter.goal, [0., 8., 2.])
        self.assertEqual(self.adapter.navigation.target_revision, 1)
        self.nav.cancel()

    def test_rejected_update_keeps_accepted_destination_revision(self):
        self.nav.to_point(0, 8, 2, wait=False)
        self.wait_for(lambda: self.adapter.enabled)
        with self.assertRaisesRegex(RuntimeError, 'INVALID_GOAL'):
            self.nav.to_point(0, 9, 2, frame_id='wrong', wait=False)
        self.assertEqual(self.nav._target_revision, 1)
        self.assertEqual(self.adapter.goal, [0., 8., 2.])
        self.nav.to_point(0, 12, 2, wait=False)
        self.assertEqual(self.nav._target_revision, 3)
        self.assertEqual(self.adapter.goal, [0., 12., 2.])
        self.nav.cancel()

    def test_hold_arrival_renew_resume_and_cancel(self):
        self.nav.to_point(0, 0, 2, wait=False, arrival_behavior='hold', target_timeout=10.)
        self.wait_for(lambda: self.adapter.enabled)
        self.adapter.client.complete = True
        self.wait_for(lambda: self.nav.feedback and self.nav.feedback.arrived)
        self.assertFalse(self.nav.done)
        handle = self.nav._handle
        self.assertTrue(self.nav.wait_until_arrived(timeout=1.))
        self.nav.to_point(0, 0, 2, wait=False)
        self.assertTrue(self.adapter.navigation.holding)
        self.nav.to_point(0, 12, 2, wait=False)
        self.assertIs(self.nav._handle, handle)
        self.assertTrue(self.adapter.enabled)
        self.assertFalse(self.adapter.navigation.holding)
        self.nav.cancel()

    def test_cancel_while_holding_and_reject_late_update(self):
        from uav_planner_interfaces.srv import UpdateNavigationTarget
        self.nav.to_point(0, 0, 2, wait=False, arrival_behavior='hold', target_timeout=10.)
        self.wait_for(lambda: self.adapter.enabled)
        self.adapter.client.complete = True
        self.wait_for(lambda: self.nav.feedback and self.nav.feedback.arrived)
        old_id = self.nav._handle.goal_id
        self.nav.cancel()
        self.assertEqual(self.nav.result.code, 'CANCELED')
        request = UpdateNavigationTarget.Request(goal_id=old_id, target_revision=2)
        response = self.nav._wait(self.nav._updates.call_async(request), 2.)
        self.assertFalse(response.accepted)
        self.assertEqual(response.code, 'NO_ACTIVE_TASK')

    def test_target_expiry_and_immutable_policies(self):
        self.nav.to_point(0, 8, 2, wait=False, arrival_behavior='hold', target_timeout=.3)
        self.wait_for(lambda: self.adapter.enabled)
        with self.assertRaises(ValueError):
            self.nav.to_point(0, 10, 2, wait=False, arrival_behavior='complete')
        self.assertEqual(self.wait_result().code, 'TARGET_TIMEOUT')
        self.assertTrue(self.nav.result.stop_confirmed)

    def test_task_timeout_uses_existing_stop(self):
        self.nav.to_point(0, 4, 2, wait=False, timeout=.15)
        result = self.wait_result()
        self.assertEqual(result.code, 'TIMEOUT')
        self.assertTrue(result.stop_confirmed)


if __name__ == '__main__':
    # Node remapping applies to both the adapter and test publisher.
    import sys
    # rclpy.init() consumes ROS arguments without unittest parsing them.
    args = sys.argv[1:]
    sys.argv[1:] = []
    original_init = rclpy.init
    rclpy.init = lambda: original_init(args=['--ros-args', '-r', '__ns:=/drone0'])
    unittest.main(verbosity=2)
