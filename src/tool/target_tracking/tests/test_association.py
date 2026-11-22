"""Check the selected raw observation against the locked, real EKF."""

import unittest

from uav_target_tracking.core import Tracker


class AssociatedPositionTests(unittest.TestCase):
    def test_filtered_position_does_not_replace_raw_observation(self):
        tracker = Tracker(position_filter='robust_linear_v1')
        for index in range(8):
            position = (2.0 + (0.08 if index == 6 else 0.0), 0.15, 1.2)
            tracker.observe([position], 1.0 + index * 0.1, 'drone0/odom')
        position = (2.14, 0.15, 1.2)
        tracker.observe([position], 1.8, 'drone0/odom')
        self.assertEqual(tracker.associated_position, position)
        self.assertNotEqual(tracker.measurement_audit['filtered'][0], list(position))

    def test_selection_loss_and_reacquisition(self):
        tracker = Tracker()
        target = (2.0, 0.15, 1.2)
        distractor = (2.8, 0.15, 1.2)
        for index in range(8):
            tracker.observe([target], 1.0 + index * 0.1, 'drone0/odom')
        tracker.observe([distractor, target], 1.8, 'drone0/odom')
        self.assertEqual(tracker.associated_position, target)
        self.assertEqual(tracker.track_epoch, 1)
        tracker.observe([], 2.0, 'drone0/odom')
        self.assertIsNone(tracker.associated_position)
        tracker.observe([], 2.5, 'drone0/odom')
        self.assertIsNone(tracker.associated_position)
        for index in range(8):
            tracker.observe([target], 2.6 + index * 0.1, 'drone0/odom')
        self.assertEqual(tracker.associated_position, target)
        self.assertEqual(tracker.track_epoch, 2)

    def test_same_id_wins_inside_gate_and_scope_change_restores_geometry(self):
        tracker = Tracker()
        for i in range(8):
            tracker.observe([(2., 0., 1.)], 1. + i * .1, 'map', ids=[1], scope=10)
        tracker.observe([(2., 0., 1.), (2.4, 0., 1.)], 1.8, 'map', ids=[2, 1], scope=10)
        self.assertEqual(tracker.associated_position, (2.4, 0., 1.))
        tracker.observe([(30., 0., 1.), (2.3, 0., 1.)], 1.9, 'map', ids=[1, 2], scope=10)
        self.assertIsNone(tracker.associated_position)
        tracker.observe([(2.4, 0., 1.)], 2.0, 'map')
        self.assertEqual(tracker.associated_position, (2.4, 0., 1.))
        tracker.observe([(2.3, 0., 1.)], 2.1, 'map', ids=[2], scope=11)
        self.assertEqual(tracker.associated_position, (2.3, 0., 1.))
        self.assertEqual(tracker.track_epoch, 1)


if __name__ == '__main__':
    unittest.main()
