"""Real ByteTrack regression for Tracking-owned history; no ROS or weights needed."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'ros2_adapter/uav_target_tracking/stream_tracker.py'
spec = importlib.util.spec_from_file_location('tracking_stream_tracker', MODULE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def frame(stamp, *, empty=False, worker=1):
    detection = NS(xyxy=[20., 20., 80., 100.], confidence=.9, category='person')
    return NS(tool_generation=1, worker_generation=worker, model_version='test-model',
              image_header=NS(frame_id='camera_optical', stamp=NS(sec=1, nanosec=stamp)),
              detections=[] if empty else [NS(detection=detection)])


class SessionTrackingTests(unittest.TestCase):
    def test_ids_survive_missing_frame_without_mutating_measurements(self):
        tracker = module.StreamTracker()
        source = frame(1)
        ids, scope = tracker.attach(source)
        self.assertGreater(ids[0], 0)
        self.assertEqual(tracker.attach(frame(2, empty=True)), ((), scope))
        self.assertEqual(tracker.attach(frame(3)), (ids, scope))
        self.assertEqual(vars(source.detections[0].detection),
                         dict(xyxy=[20., 20., 80., 100.], confidence=.9, category='person'))
        with self.assertRaises(ValueError):
            tracker.attach(frame(3))
        tracker.close()
        self.assertIsNone(tracker.tracker)
        self.assertIsNone(tracker.key)

    def test_sessions_and_producer_restarts_have_separate_history(self):
        first = module.StreamTracker()
        _, scope = first.attach(frame(1))
        _, restarted_scope = first.attach(frame(1, worker=2))
        self.assertGreater(restarted_scope, scope)
        first.close()
        second = module.StreamTracker()
        self.assertIsNone(second.tracker)
        ids, new_scope = second.attach(frame(1))
        self.assertGreater(ids[0], 0)
        self.assertGreater(new_scope, restarted_scope)
        second.close()


if __name__ == '__main__':
    unittest.main()
