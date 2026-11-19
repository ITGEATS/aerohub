"""Run with the real ROS/PX4 diagnostic overlay; no publishers or flight calls."""
import json
import tempfile
import unittest

import rclpy
from px4_msgs.msg import TrajectorySetpoint

from flight import Flight


class FlightEvidenceTest(unittest.TestCase):
    def test_real_px4_array_callback_survives_failed_flight_save(self):
        rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as directory:
                node = Flight(directory)
                try:
                    message = TrajectorySetpoint()
                    message.position = [1.25, -2.5, 3.0]
                    message.velocity = [0.0, 0.125, -0.5]
                    subscriptions = [s for s in node.subscriptions
                                     if s.topic_name.endswith('/fmu/in/trajectory_setpoint')]
                    self.assertEqual(len(subscriptions), 1)
                    subscriptions[0].callback(message)
                    node.result['takeoff_success'] = False
                    node.save()
                    events = json.loads((node.output/'flight-events.json').read_text())
                    self.assertEqual(events[0]['position'], [1.25, -2.5, 3.0])
                    self.assertEqual(events[0]['velocity'], [0.0, 0.125, -0.5])
                    self.assertFalse(json.loads((node.output/'flight-result.json').read_text())['takeoff_success'])
                finally:
                    node.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
