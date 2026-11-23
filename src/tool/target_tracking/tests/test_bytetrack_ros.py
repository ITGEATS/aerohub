"""Actual ROS/ByteTrack session isolation and cleanup; synthetic detection inputs."""
import time
from unittest.mock import patch

import rclpy
from uav_target_tracking.stream_tracker import StreamTracker
from test_stream_ros import StreamTests, wait


class ByteTrackSessionTests(StreamTests):
    @classmethod
    def setUpClass(cls):
        # Import optional dependencies before starting the short timed ROS goals.
        StreamTracker().close()
        rclpy.init(args=['--ros-args', '-r', '__ns:=/bytetrack_test',
                        '-p', 'observation_mode:=stream', '-p', 'stream_query:=person',
                        '-p', 'bytetrack_enabled:=true'])

    def publish(self, worker=1):
        # Fill the actual detector contract; shared stream tests otherwise need only 3D.
        original = self.publisher.publish

        def populated(message):
            for item in message.detections:
                item.detection.xyxy = [20., 20., 80., 100.]
                item.detection.confidence = .9
                item.detection.category = 'person'
            original(message)

        with patch.object(self.publisher, 'publish', populated):
            return super().publish(worker)

    def test_cancel_releases_history_before_next_session(self):
        instances = []

        def create():
            tracker = StreamTracker()
            instances.append(tracker)
            return tracker

        with patch('uav_target_tracking.node.StreamTracker', side_effect=create):
            first = self.start()
            for _ in range(12):
                self.publish()
                time.sleep(.05)
            self.assertTrue(any(state.valid for state in self.feedback))
            self.assertIsNotNone(instances[0].tracker)
            old_scope = instances[0].generation
            wait(first.cancel_goal_async())
            wait(first.get_result_async())
            self.assertIsNone(instances[0].tracker)
            self.feedback.clear()
            second = self.start()
            for _ in range(12):
                self.publish()
                time.sleep(.05)
            self.assertEqual(len(instances), 2)
            self.assertGreater(instances[1].generation, old_scope)
            self.assertTrue(any(state.valid for state in self.feedback))
            wait(second.cancel_goal_async())
            wait(second.get_result_async())
            self.assertIsNone(instances[1].tracker)
