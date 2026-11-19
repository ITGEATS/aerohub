"""Pure logic checks; no ROS, simulator or navigation commands."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

spec = importlib.util.spec_from_file_location('tracking_mission', Path(__file__).with_name('mission.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def stamp(value):
    return NS(sec=int(value), nanosec=int((value % 1) * 1e9))


class SafetyTests(unittest.TestCase):
    def test_geometry(self):
        self.assertEqual(m.follow_point((0, 0, 7), (6, 8, -2), 5), (3, 4, 7))
        for target in [(0, 0, -10), (3, 4, 0), (1, 1, 0)]:
            self.assertEqual(m.follow_point((0, 0, 7), target, 5), (0, 0, 7))
        self.assertEqual(m.follow_point((10, 10, 4), (0, 10, 0), 5), (5, 10, 4))

    def test_bad_geometry(self):
        for distance in [-1, float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                m.follow_point((0, 0, 1), (1, 2, 0), distance)

    def test_freshness(self):
        self.assertTrue(m.fresh(10, 20, 10.5, 20.5, 1))
        for values in [(10, 20, 12, 20.5, 1), (10, 20, 10, 22, 1),
                       (10, 20, 9, 20, 1), (10, 20, 10, 19, 1),
                       (0, 20, 0, 20, 1)]:
            self.assertFalse(m.fresh(*values))

    def test_tracker_gates(self):
        state = NS(valid=True, header=NS(frame_id='earth', stamp=stamp(10)),
                   observation_stamp=stamp(9), valid_until=stamp(11), status='tracked', capture_id=4)
        self.assertTrue(m.tracking_valid(state, 20, 10, 20, 'earth', 1))
        self.assertFalse(m.tracking_valid(state, 20, 11, 20, 'earth', 1))
        self.assertFalse(m.tracking_valid(state, 20, 10, 22, 'earth', 1))
        self.assertFalse(m.tracking_valid(state, 20, 10, 20, 'odom', 1))
        state.valid = False
        self.assertFalse(m.tracking_valid(state, 20, 10, 20, 'earth', 1))

    def test_diagnostics_preserve_future_and_expiry_guards(self):
        state = NS(valid=True, header=NS(frame_id='earth', stamp=stamp(10)),
                   observation_stamp=stamp(9.8), valid_until=stamp(10.3), status='tracked', capture_id=7)
        future = m.tracking_diagnostics(state, 20, 9.99, 20.01, 'earth', 1)
        self.assertFalse(future['predicates']['header_fresh'])
        self.assertTrue(future['predicates']['not_expired'])
        self.assertFalse(m.tracking_valid(state, 20, 9.99, 20.01, 'earth', 1))
        expired = m.tracking_diagnostics(state, 20, 10.3, 20.3, 'earth', 1)
        self.assertFalse(expired['predicates']['not_expired'])
        self.assertTrue(expired['predicates']['header_fresh'])
        self.assertFalse(m.tracking_valid(state, 20, 10.3, 20.3, 'earth', 1))

    def test_clock_regression_latched(self):
        box = m.Mailbox()
        for value in [10, 9, 11]:
            box.clock(NS(clock=stamp(value)))
        self.assertTrue(box.snapshot()[1])

    def test_observe_default(self):
        args = m.arguments([])
        self.assertFalse(args.execute_navigation)
        self.assertEqual((args.rate, args.distance, args.max_age, args.target_timeout), (2, 5, 1, 3))
        self.assertTrue(m.arguments(['--execute-navigation']).execute_navigation)


if __name__ == '__main__':
    unittest.main()
