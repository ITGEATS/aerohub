"""Real ROS/DDS contract tests with synthetic sensors and a fake executor.

No cloud, simulator, PX4 or flight acceptance is exercised. Run with the rebuilt
uav_planner_interfaces overlay, in an isolated ROS domain.
"""
import json
import threading
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_planner_interfaces.action import FollowTimedPath, NavigateToSemanticGoal
from uav_spf_planner.node import SpfPlannerNode
from uav_inference_interfaces.action import InferVisionLanguage


class SemanticNavigationContract(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.planner = SpfPlannerNode()
        self.driver = Node('semantic_navigation_test')
        self.stop_motion = threading.Event()
        self.motion_started = threading.Event()
        self.reply_mode = 'silent'
        self.last_query = None
        self.accept_delay = 0.0
        self.complete_motion = False
        self.fresh_twist = True
        self.stop_ack = True
        self.twist_z = 0.0
        self.goal_count = 0
        self.stop_count = 0
        self.feedback = []
        self.group = ReentrantCallbackGroup()
        self.execution = ActionServer(
            self.driver, FollowTimedPath, 'follow_timed_path',
            execute_callback=self.execute_motion, goal_callback=self.accept_motion,
            callback_group=self.group)
        self.driver.create_service(Trigger, 'follow_timed_path/_behavior/stop',
                                   self.stop, callback_group=self.group)
        self.pose = self.driver.create_publisher(PoseStamped, 'self_localization/pose', 10)
        self.twist = self.driver.create_publisher(TwistStamped, 'self_localization/twist', 10)
        self.camera = self.driver.create_publisher(
            CameraInfo, 'sensor_measurements/FrontCamera/camera_info', 10)
        self.inference = ActionServer(
            self.driver, InferVisionLanguage, 'inference/vision_language/infer',
            execute_callback=self.infer, cancel_callback=lambda h: CancelResponse.ACCEPT,
            callback_group=self.group)
        self.driver.create_timer(0.025, self.sensors)
        self.client = ActionClient(self.driver, NavigateToSemanticGoal,
                                   'navigate_to_semantic_goal')
        self.executor = MultiThreadedExecutor(num_threads=5)
        self.executor.add_node(self.planner)
        self.executor.add_node(self.driver)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.assertTrue(self.client.wait_for_server(timeout_sec=5.0))
        deadline = time.monotonic() + 5
        while not self.planner._ready() and time.monotonic() < deadline:
            time.sleep(.025)
        self.assertTrue(self.planner._ready())

    def tearDown(self):
        self.stop_motion.set()
        # Drain executor callbacks before invalidating publishers' ROS context.
        self.executor.shutdown(timeout_sec=12)
        self.thread.join(timeout=2)
        rclpy.shutdown()
        self.planner._worker.join(timeout=12)
        self.planner._semantic.destroy()
        self.execution.destroy()
        self.inference.destroy()
        self.client.destroy()
        self.planner.destroy_node()
        self.driver.destroy_node()

    def sensors(self):
        stamp = self.driver.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'drone0/odom'
        pose.pose.position.z = 1.5
        pose.pose.orientation.w = 1.0
        self.pose.publish(pose)
        info = CameraInfo()
        info.header.stamp = stamp
        info.width = info.height = 256
        info.k = [128., 0., 128., 0., 128., 128., 0., 0., 1.]
        self.camera.publish(info)
        if self.fresh_twist:
            twist = TwistStamped()
            twist.header.stamp = stamp
            twist.twist.linear.z = self.twist_z
            self.twist.publish(twist)

    def infer(self, handle):
        from std_msgs.msg import Header
        self.last_query = handle.request
        while self.reply_mode == 'silent' and rclpy.ok():
            if handle.is_cancel_requested:
                handle.canceled()
                return InferVisionLanguage.Result(error_code='CANCELED')
            time.sleep(.025)
        result = InferVisionLanguage.Result(
            success=self.reply_mode in ('point', 'malformed'),
            model_name=handle.request.model_name,
            text='[{"point": [500, 500], "depth": 5}]',
            observation_header=Header(stamp=self.driver.get_clock().now().to_msg(),
                                      frame_id='drone0/FrontCamera_optical'))
        if self.reply_mode == 'malformed':
            result.text = '[{"point": null, "depth": 5}]'
        if result.success:
            handle.succeed()
        else:
            result.error_code = 'TEST_FAILURE'
            handle.abort()
        return result

    def accept_motion(self, request):
        time.sleep(self.accept_delay)
        return GoalResponse.ACCEPT

    def execute_motion(self, handle):
        self.stop_motion.clear()
        self.goal_count += 1
        if self.complete_motion:
            handle.succeed()
            return FollowTimedPath.Result(success=True)
        self.motion_started.set()
        self.stop_motion.wait(20)
        handle.abort()
        return FollowTimedPath.Result(success=False, reason='test stop')

    def stop(self, request, response):
        self.stop_count += 1
        if self.stop_ack:
            self.stop_motion.set()
        response.success = self.stop_ack
        return response

    def wait(self, future, timeout=15):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(future.done(), 'ROS future did not complete')
        return future.result()

    def start(self, description='approach the orange ball', timeout=0.4):
        goal = NavigateToSemanticGoal.Goal(description=description, timeout=timeout)
        handle = self.wait(self.client.send_goal_async(
            goal, feedback_callback=lambda msg: self.feedback.append(msg.feedback)))
        self.assertTrue(handle.accepted)
        return handle

    def test_invalid_goals_are_rejected(self):
        for description, timeout in [('', 1.), ('  ', 1.), ('ball', 0.),
                                     ('ball', -1.), ('ball', float('nan')),
                                     ('ball', float('inf'))]:
            goal = NavigateToSemanticGoal.Goal(description=description, timeout=timeout)
            self.assertFalse(self.wait(self.client.send_goal_async(goal)).accepted)

    def test_timeout_interrupts_pending_vlm_and_never_succeeds(self):
        began = time.monotonic()
        handle = self.start()
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'TIMEOUT')
        self.assertFalse(result.success)
        self.assertTrue(result.stop_confirmed)
        self.assertEqual(self.goal_count, 0)
        self.assertLess(time.monotonic() - began, 4.)
        self.assertTrue(self.feedback)
        # Ownership is released after stop confirmation, allowing a next task.
        second = self.start(timeout=0.2)
        self.assertEqual(self.wait(second.get_result_async()).result.code, 'TIMEOUT')

    def test_cancel_pending_vlm_rejects_concurrent_goal(self):
        handle = self.start(timeout=20.)
        goal = NavigateToSemanticGoal.Goal(description='another goal', timeout=1.)
        self.assertFalse(self.wait(self.client.send_goal_async(goal)).accepted)
        self.wait(handle.cancel_goal_async())
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'CANCELED')
        self.assertTrue(result.stop_confirmed)
        self.assertEqual(self.goal_count, 0)
        # Canceled inference remains attached to its original Action handle.
        time.sleep(.15)
        self.assertEqual(self.goal_count, 0)

    def test_timeout_stops_active_trajectory(self):
        self.reply_mode = 'point'
        handle = self.start(timeout=1.)
        self.assertTrue(self.motion_started.wait(3))
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'TIMEOUT')
        self.assertTrue(result.stop_confirmed)
        self.assertFalse(result.success)
        self.assertEqual(self.goal_count, 1)
        self.assertGreater(self.stop_count, 0)

    def test_segment_success_does_not_finish_semantic_goal(self):
        self.reply_mode = 'point'
        self.complete_motion = True
        handle = self.start(timeout=0.8)
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'TIMEOUT')
        self.assertFalse(result.success)
        self.assertGreater(self.goal_count, 1)
        self.assertTrue(any(f.steps_completed > 0 for f in self.feedback))

    def test_late_execution_acceptance_is_stopped_before_release(self):
        self.reply_mode = 'point'
        self.accept_delay = 0.8
        handle = self.start(timeout=0.3)
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'TIMEOUT')
        self.assertTrue(result.stop_confirmed)
        self.assertEqual(self.goal_count, 1)
        self.assertGreaterEqual(self.stop_count, 2)

    def test_legacy_debug_goal_cannot_replace_action_or_deadlock(self):
        handle = self.start(timeout=0.5)
        publisher = self.driver.create_publisher(String, 'spf/goal', 10)
        publisher.publish(String(data=json.dumps(dict(instruction='replace', max_cycles=1))))
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'TIMEOUT')
        self.assertTrue(result.stop_confirmed)

    def test_timeout_interrupts_retry_backoff(self):
        self.reply_mode = 'failure'
        began = time.monotonic()
        handle = self.start()
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'TIMEOUT')
        self.assertLess(time.monotonic() - began, 4.)

    def test_malformed_model_output_fails_without_killing_worker(self):
        self.reply_mode = 'malformed'
        handle = self.start(timeout=5.)
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'EXECUTION_FAILED')
        self.assertTrue(result.stop_confirmed)
        self.reply_mode = 'silent'
        second = self.start()
        self.assertEqual(self.wait(second.get_result_async()).result.code, 'TIMEOUT')

    def test_stale_twist_cannot_confirm_stop(self):
        self.fresh_twist = False
        handle = self.start()
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'STOP_UNCONFIRMED')
        self.assertFalse(result.stop_confirmed)
        goal = NavigateToSemanticGoal.Goal(description='ball', timeout=1.)
        self.assertFalse(self.wait(self.client.send_goal_async(goal)).accepted)

    def test_vertical_motion_cannot_confirm_stop(self):
        self.twist_z = 1.0
        handle = self.start()
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'STOP_UNCONFIRMED')
        self.assertFalse(result.stop_confirmed)

    def test_rejected_stop_cannot_confirm_stop(self):
        self.stop_ack = False
        handle = self.start()
        result = self.wait(handle.get_result_async()).result
        self.assertEqual(result.code, 'STOP_UNCONFIRMED')
        self.assertFalse(result.stop_confirmed)


if __name__ == '__main__':
    unittest.main(verbosity=2)
