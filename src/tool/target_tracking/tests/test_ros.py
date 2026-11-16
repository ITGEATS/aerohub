"""Real DDS action/service harness with explicitly synthetic observations."""
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from uav_object_detection_interfaces.msg import ObjectDetection
from uav_object_detection_interfaces.srv import DetectSnapshot
from uav_target_tracking_interfaces.action import TrackObject
from uav_target_tracking.node import TrackingNode


def wait(future, timeout=5):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline: time.sleep(.005)
    assert future.done(), 'future timeout'
    return future.result()


class RosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.tracker = TrackingNode()
        cls.peer = Node('tracking_test_peer')
        cls.delay = .03
        cls.busy = 0
        cls.peak = 0
        cls.calls = 0
        cls.lock = threading.Lock()
        cls.bad_stamp = False
        cls.no_position = False
        cls.future_stamp = False
        cls.no_detection = False
        def snapshot(request, response):
            assert request.estimate_position
            assert request.query == 'person'
            assert not hasattr(request, 'category')
            with cls.lock:
                cls.busy += 1; cls.peak = max(cls.busy, cls.peak); cls.calls += 1
            try:
                time.sleep(cls.delay)
                response.success = True
                response.result.position_requested = True
                response.result.header.frame_id = 'map'
                response.result.header.stamp = cls.tracker.get_clock().now().to_msg()
                if cls.bad_stamp: response.result.header.stamp.sec -= 2
                if cls.future_stamp: response.result.header.stamp.sec += 2
                d = ObjectDetection(position_valid=not cls.no_position)
                d.position.x = 2.; d.position.y = 1.; d.position.z = 1.
                response.result.detections = [] if cls.no_detection else [d]
                return response
            finally:
                with cls.lock: cls.busy -= 1
        cls.service = cls.peer.create_service(DetectSnapshot, 'tools/object_detection/observe', snapshot, callback_group=ReentrantCallbackGroup())
        cls.client = ActionClient(cls.peer, TrackObject, 'tools/target_tracking/track')
        cls.executor = MultiThreadedExecutor(num_threads=6)
        cls.executor.add_node(cls.tracker); cls.executor.add_node(cls.peer)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True); cls.thread.start()
        assert cls.client.wait_for_server(timeout_sec=5)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=5); cls.thread.join(5)
        cls.client.destroy(); cls.tracker.destroy_node(); cls.peer.destroy_node(); rclpy.shutdown()

    def goal(self, timeout, feedback=None, query='person'):
        return wait(self.client.send_goal_async(TrackObject.Goal(query=query, timeout=float(timeout)), feedback_callback=feedback))

    def test_01_timeout_feedback_and_single_session(self):
        feedback = []
        h = self.goal(.8, lambda msg: feedback.append(msg.feedback.state))
        self.assertTrue(h.accepted)
        self.assertFalse(self.goal(.2).accepted)
        result = wait(h.get_result_async())
        self.assertEqual(result.result.code, 'TIMEOUT')
        self.assertEqual(result.status, 4)
        self.assertTrue(any(s.valid and s.status == 'tracked' for s in feedback))
        self.assertTrue(all(s.status == 'tracked' for s in feedback if s.valid and s.reason == 'tracked'))
        self.assertTrue(all(s.session_id == feedback[0].session_id for s in feedback))
        self.assertTrue(any(s.capture_id > 1 for s in feedback))
        self.assertFalse(result.result.state.valid)
        self.assertEqual(self.peak, 1)

    def test_02_cancel_late_reply_generation(self):
        type(self).delay = .4
        h = self.goal(2)
        end = time.monotonic() + 2
        while not self.busy and time.monotonic() < end: time.sleep(.005)
        started = time.monotonic()
        self.assertTrue(wait(h.cancel_goal_async()).goals_canceling)
        first = wait(h.get_result_async())
        self.assertLess(time.monotonic() - started, .3)
        self.assertEqual(first.result.code, 'CANCELED')
        feedback = []
        next_h = self.goal(.12, lambda msg: feedback.append(msg.feedback.state))
        next_result = wait(next_h.get_result_async())
        self.assertNotEqual(first.result.state.session_id, next_result.result.state.session_id)
        self.assertTrue(all(s.capture_id == 0 and not s.valid for s in feedback))
        time.sleep(.5)
        type(self).delay = .03
        self.assertEqual(self.peak, 1)

    def test_03_invalid_goal_and_stale_capture(self):
        self.assertFalse(hasattr(TrackObject.Goal(),'category'))
        self.assertFalse(self.goal(.1,query=' ').accepted)
        for timeout in [0., -1., float('nan'), float('inf')]: self.assertFalse(self.goal(timeout).accepted)
        type(self).bad_stamp = True
        feedback = []
        h = self.goal(.3, lambda msg: feedback.append(msg.feedback.state))
        self.assertEqual(wait(h.get_result_async()).result.code, 'TIMEOUT')
        self.assertTrue(all(not s.valid and s.capture_id == 0 for s in feedback))
        type(self).bad_stamp = False

    def test_035_unavailable_geometry_and_future(self):
        for attribute in ('no_position', 'future_stamp'):
            setattr(type(self), attribute, True)
            feedback = []
            h = self.goal(.25, lambda msg: feedback.append(msg.feedback.state))
            self.assertEqual(wait(h.get_result_async()).result.code, 'TIMEOUT')
            self.assertTrue(feedback)
            self.assertTrue(all(s.capture_id == 0 and not s.valid for s in feedback))
            setattr(type(self), attribute, False)
        type(self).no_detection = True
        feedback = []
        h = self.goal(.3, lambda msg: feedback.append(msg.feedback.state))
        wait(h.get_result_async())
        self.assertTrue(any(s.capture_id > 0 for s in feedback))
        self.assertTrue(all(not s.valid for s in feedback))
        type(self).no_detection = False

    def test_04_clock_reset(self):
        # Set ROS clock explicitly; finite wall timeout remains independent.
        self.tracker.set_parameters([Parameter('use_sim_time', value=True)])
        pub = self.peer.create_publisher(Clock, '/clock', 10)
        clock = Clock(); clock.clock.sec = 100
        for _ in range(10): pub.publish(clock); time.sleep(.02)
        h = self.goal(2)
        clock.clock.sec = 90
        for _ in range(10): pub.publish(clock); time.sleep(.02)
        result = wait(h.get_result_async())
        self.assertGreater(self.tracker._clock_epoch, 0)
        self.assertEqual(result.result.code, 'CLOCK_RESET')
        self.assertEqual(result.status, 6)
        self.peer.destroy_publisher(pub)

if __name__ == '__main__': unittest.main()
