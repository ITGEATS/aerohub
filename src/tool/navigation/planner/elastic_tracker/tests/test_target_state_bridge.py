import unittest
from types import SimpleNamespace

import rclpy
from uav_target_tracking_interfaces.msg import TargetState

from uav_elastic_tracker_adapter.elastic_tracking_bridge import TargetStateBridge


def state(stamp, *, session='first', generation=1, valid=True, status='tracked', expires=10):
    message = TargetState(session_id=session, status=status, valid=valid, generation=generation)
    message.header.frame_id = 'odom'
    message.header.stamp.sec = stamp
    message.observation_stamp.sec = 1
    message.valid_until.sec = expires
    message.position.x = 3.
    message.velocity.y = -.7
    message.covariance = [float(value) for value in range(36)]
    return message


class TargetStateBridgeTest(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.bridge = TargetStateBridge()
        self.bridge.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=3_000_000_000))
        self.outputs = []
        self.bridge.output = SimpleNamespace(publish=self.outputs.append)

    def tearDown(self):
        self.bridge.destroy_node()
        rclpy.shutdown()

    def test_copies_odometry_contract(self):
        source = state(3, status='predicted')
        self.bridge.forward(source)
        result, = self.outputs
        self.assertEqual(result.header, source.header)
        self.assertEqual(result.child_frame_id, 'odom')
        self.assertEqual(result.pose.pose.position, source.position)
        self.assertEqual(result.pose.pose.orientation.w, 1.)
        self.assertEqual(result.twist.twist.linear, source.velocity)
        for row in range(3):
            for column in range(3):
                self.assertEqual(result.pose.covariance[6 * row + column],
                                 source.covariance[6 * row + column])
                self.assertEqual(result.twist.covariance[6 * row + column],
                                 source.covariance[6 * (row + 3) + column + 3])

    def test_invalid_duplicate_old_generation_and_stopped_are_silent(self):
        wrong_frame = state(1)
        wrong_frame.header.frame_id = 'camera'
        messages = [
            wrong_frame, state(1, valid=False, status='confirm'), state(2, expires=2),
            state(2), state(2), state(3, valid=False, status='lost'),
            state(1, generation=2), state(4), state(5, generation=2, valid=False, status='stopped'),
            state(6, generation=2), state(7, session='second'),
        ]
        for message in messages:
            self.bridge.forward(message)
        self.assertEqual([message.header.stamp.sec for message in self.outputs], [2, 1, 7])


if __name__ == '__main__':
    unittest.main()
