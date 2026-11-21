"""Observation-only ROS sessions reject motion even with Navigation available."""
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from uav_object_detection_interfaces.msg import ObjectDetection
from uav_object_detection_interfaces.srv import DetectSnapshot
from uav_planner_interfaces.action import NavigateToPose
from uav_planner_interfaces.srv import UpdateNavigationTarget
from uav_target_tracking.node import TrackingNode
from uav_target_tracking_interfaces.action import TrackObject


def wait(future, timeout=6):
    deadline = time.monotonic()+timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise TimeoutError('ROS follow test deadline')
        time.sleep(.01)
    return future.result()


class FollowRosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.tracker = TrackingNode()
        cls.peer = Node('follow_test_peer')
        cls.group = ReentrantCallbackGroup()
        cls.peer.create_service(DetectSnapshot,'tools/object_detection/observe',cls.snapshot,callback_group=cls.group)
        cls.client = ActionClient(cls.peer,TrackObject,'tools/target_tracking/track')
        cls.executor = MultiThreadedExecutor(num_threads=8)
        cls.executor.add_node(cls.tracker)
        cls.executor.add_node(cls.peer)
        cls.thread = threading.Thread(target=cls.executor.spin,daemon=True)
        cls.thread.start()
        assert cls.client.wait_for_server(timeout_sec=5)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=5)
        cls.thread.join(5)
        cls.client.destroy()
        cls.peer.destroy_node()
        cls.tracker.destroy_node()
        rclpy.shutdown()

    @classmethod
    def snapshot(cls, request, response):
        time.sleep(.025)
        response.success = True
        response.result.position_requested = True
        response.result.header.frame_id = 'map'
        response.result.header.stamp = cls.peer.get_clock().now().to_msg()
        detection = ObjectDetection(position_valid=True)
        detection.position.x, detection.position.y, detection.position.z = 2.,1.,1.
        response.result.detections = [detection]
        return response

    def goal(self, follow, timeout=1.2):
        handle = wait(self.client.send_goal_async(TrackObject.Goal(query='person',timeout=timeout,follow=follow)))
        self.assertTrue(handle.accepted)
        return handle

    def test_01_default_false_needs_no_navigation(self):
        feedback = []
        handle = wait(self.client.send_goal_async(
            TrackObject.Goal(query='person', timeout=.5, follow=False),
            feedback_callback=lambda message: feedback.append(message.feedback.state)))
        self.assertTrue(handle.accepted)
        result = wait(handle.get_result_async()).result
        self.assertTrue(feedback)
        self.assertTrue(any(state.valid for state in feedback))
        self.assertEqual(result.code,'TIMEOUT')
        self.assertTrue(result.stop_confirmed)

    def navigation_peer(self, unsupported=False):
        self.goals = []
        self.updates = []
        def execute(handle):
            self.goals.append(handle.request)
            if unsupported:
                handle.abort()
                return NavigateToPose.Result(code='UNSUPPORTED',reason='yaw unsupported',stop_confirmed=True)
            while not handle.is_cancel_requested:
                time.sleep(.01)
            handle.canceled()
            return NavigateToPose.Result(code='CANCELED',reason='stopped',stop_confirmed=True)
        def update(request,response):
            self.updates.append(request)
            response.accepted = True
            response.target_revision = request.target_revision
            return response
        server = ActionServer(self.peer,NavigateToPose,'navigate_to_pose',execute,
            callback_group=self.group,cancel_callback=lambda _:CancelResponse.ACCEPT)
        updates = self.peer.create_service(UpdateNavigationTarget,'navigate_to_pose/update_target',update,callback_group=self.group)
        self.addCleanup(server.destroy)
        self.addCleanup(lambda:self.peer.destroy_service(updates))
        time.sleep(.2)

    def test_02_follow_true_is_rejected_even_with_navigation_available(self):
        self.navigation_peer()
        handle = wait(self.client.send_goal_async(TrackObject.Goal(query='person', timeout=1.2, follow=True)))
        self.assertFalse(handle.accepted)
        self.assertEqual(wait(self.goal(False, .5).get_result_async()).result.code, 'TIMEOUT')
        self.assertEqual(self.goals, [])
        self.assertEqual(self.updates, [])


if __name__=='__main__':
    unittest.main()
