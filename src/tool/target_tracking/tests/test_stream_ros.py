"""Actual DDS stream consumption with no snapshot service; not flight evidence."""
import threading
import time
import unittest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from uav_target_tracking.node import TrackingNode
from uav_target_tracking_interfaces.action import TrackObject
from uav_target_tracking_interfaces.msg import AssociatedPosition3D
from uav_object_detection_interfaces.msg import ObjectDetectionFrame, ObjectDetection


def wait(future, timeout=5.):
    end=time.monotonic()+timeout
    while not future.done():
        if time.monotonic()>end:raise TimeoutError('ROS future deadline')
        time.sleep(.01)
    return future.result()


class StreamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args','-r','__ns:=/stream_test',
            '-p','observation_mode:=stream','-p','stream_query:=person'])

    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def setUp(self):
        self.tool=TrackingNode();self.peer=Node('stream_peer')
        self.publisher=self.peer.create_publisher(ObjectDetectionFrame,'perception/object_detection/detections',1)
        self.client=ActionClient(self.peer,TrackObject,'tools/target_tracking/track')
        self.observations = []
        self.peer.create_subscription(AssociatedPosition3D, 'tools/target_tracking/associated_position',
                                      self.observations.append, 10)
        self.executor=MultiThreadedExecutor(num_threads=6)
        for node in(self.tool,self.peer):self.executor.add_node(node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        self.assertTrue(self.client.wait_for_server(timeout_sec=5.))
        self.feedback=[]

    def tearDown(self):
        self.executor.shutdown(timeout_sec=5.);self.thread.join(5.)
        self.client.destroy();self.tool.destroy_node();self.peer.destroy_node()

    def publish(self,worker=1):
        msg=ObjectDetectionFrame(valid=True,tool_generation=1,worker_generation=worker,model_version='test',
                                 tracking_generation=3)
        msg.header.stamp=self.peer.get_clock().now().to_msg();msg.header.frame_id='map'
        msg.image_header=msg.header
        detection=ObjectDetection(position_valid=True)
        detection.position.x=8.;detection.position.z=1.
        detection.detection.track_id = 1
        msg.detections=[detection];self.publisher.publish(msg)
        return msg.header.stamp

    def start(self):
        handle=wait(self.client.send_goal_async(TrackObject.Goal(query='person',timeout=3.),
            feedback_callback=lambda m:self.feedback.append(m.feedback.state)))
        self.assertTrue(handle.accepted)
        return handle

    def test_stream_tracks_without_snapshot_and_expires_when_producer_stops(self):
        handle=self.start()
        stamps = []
        for _ in range(16):
            stamps.append(self.publish())
            time.sleep(.05)
        time.sleep(.1)  # drain the final publication before measuring absence
        self.assertTrue(any(s.valid for s in self.feedback))
        last_valid=[s for s in self.feedback if s.valid][-1]
        self.assertTrue(last_valid.velocity_valid)
        self.assertTrue(self.observations)
        captured = len(self.observations)
        for event in self.observations:
            self.assertIn(event.header.stamp, stamps)
            self.assertEqual(event.header.frame_id, 'map')
            self.assertEqual(event.position.x, 8.)
            self.assertEqual(event.session_id, last_valid.session_id)
            self.assertEqual(event.generation, last_valid.generation)
            self.assertEqual(event.track_epoch, 1)
        self.assertEqual(self.observations[-1].capture_id, last_valid.capture_id)
        time.sleep(.65)
        self.assertFalse(self.feedback[-1].valid)
        self.assertFalse(self.feedback[-1].velocity_valid)
        self.assertEqual(len(self.observations), captured)
        self.assertEqual(self.feedback[-1].observation_stamp,last_valid.observation_stamp)
        wait(handle.cancel_goal_async())
        self.assertTrue(wait(handle.get_result_async()).result.stop_confirmed)
        self.assertIsNone(self.tool._pending)  # no snapshot peer exists in this test

    def test_producer_generation_change_ends_session(self):
        handle=self.start()
        for _ in range(12):self.publish();time.sleep(.05)
        self.publish(worker=2)
        result=wait(handle.get_result_async()).result
        self.assertEqual(result.code,'OBSERVATION_GENERATION_CHANGED')
        self.assertTrue(result.stop_confirmed)

    def test_different_query_rejected_in_fixed_query_stream(self):
        handle=wait(self.client.send_goal_async(TrackObject.Goal(query='car',timeout=1.)))
        self.assertFalse(handle.accepted)


if __name__=='__main__':unittest.main()
