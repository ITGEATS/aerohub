"""Real ROS transport for follow, with synthetic observations and Navigation peer."""
import math
import threading
import time
import unittest

import rclpy
from as2_msgs.msg import PlatformInfo, PlatformStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
        cls.pose = cls.peer.create_publisher(PoseStamped,'self_localization/pose',qos_profile_sensor_data)
        cls.platform = cls.peer.create_publisher(PlatformInfo,'platform/info',qos_profile_sensor_data)
        cls.peer.create_timer(.02,cls.publish)
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
    def publish(cls):
        stamp = cls.peer.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'map',stamp
        pose.pose.position.z, pose.pose.orientation.w = 3.,1.
        cls.pose.publish(pose)
        platform = PlatformInfo(connected=True,armed=True,offboard=True)
        platform.header.stamp = stamp
        platform.status.state = PlatformStatus.FLYING
        cls.platform.publish(platform)

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
        result = wait(self.goal(False,.5).get_result_async()).result
        self.assertEqual(result.code,'TIMEOUT')
        self.assertTrue(result.stop_confirmed)

    def test_02_follow_true_reports_missing_navigation(self):
        result = wait(self.goal(True).get_result_async()).result
        self.assertEqual(result.code,'NAVIGATION_NOT_READY')
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

    def test_03_follow_preserves_position_and_drains_navigation(self):
        self.navigation_peer()
        result = wait(self.goal(True).get_result_async()).result
        self.assertEqual(result.code,'TIMEOUT')
        self.assertTrue(result.stop_confirmed)
        self.assertEqual(len(self.goals),1)
        self.assertGreater(len(self.updates),0)
        goal = self.goals[0]
        self.assertEqual(goal.orientation_mode,goal.TARGET_YAW)
        self.assertEqual(goal.arrival_behavior,goal.HOLD)
        for target in [goal.target]+[u.target for u in self.updates]:
            self.assertEqual((target.pose.position.x,target.pose.position.y,target.pose.position.z),(0.,0.,3.))
            q = target.pose.orientation
            self.assertAlmostEqual(2*math.atan2(q.z,q.w),math.atan2(1,2))

    def test_04_unsupported_yaw_is_not_silently_ignored(self):
        self.navigation_peer(unsupported=True)
        result = wait(self.goal(True).get_result_async()).result
        self.assertEqual(result.code,'NAVIGATION_UNSUPPORTED')
        self.assertTrue(result.stop_confirmed)


if __name__=='__main__':
    unittest.main()
