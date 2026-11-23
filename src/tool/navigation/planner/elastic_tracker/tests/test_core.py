"""Synthetic structural tests, not algorithm or platform validation."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'ros2_adapter/bridge'))
from uav_elastic_tracker_adapter.core import suffix, unwrap, split_ns, stamp_ns, yaw_at


class Conversion(unittest.TestCase):
    def setUp(self):
        # Exact rational polynomial fixture: x=t in both one-second pieces.
        self.native = dict(start_time=[10, 0], hover=False, order=5,
                           duration=[1.0, 1.0], yaw=0.0,
                           coef_x=[0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 1., 1.],
                           coef_y=[0.] * 12, coef_z=[0.] * 12)

    def test_suffix_keeps_native_epoch_and_boundaries(self):
        rows = suffix(self.native, 10_500_000_000, 0.5, 0.0)
        self.assertEqual([r['time_ns'] for r in rows], [0, 500_000_000, 1_000_000_000, 1_500_000_000])
        self.assertEqual([r['pva'][0][0] for r in rows], [0.5, 1., 1.5, 2.])
        self.assertTrue(all(r['pva'][1] == [1., 0., 0.] for r in rows))

    def test_nonuniform_piece_boundary(self):
        self.native['duration'] = [0.75, 1.25]
        rows = suffix(self.native, 10_500_000_000, 0.5, 0.0)
        self.assertIn(250_000_000, [r['time_ns'] for r in rows])

    def test_no_retiming_of_expired_plan(self):
        with self.assertRaises(ValueError):
            suffix(self.native, 12_000_000_000, 0.5, 0.)

    def test_no_extrapolation_before_start(self):
        with self.assertRaises(ValueError):
            suffix(self.native, 9_000_000_000, 0.5, 0.)

    def test_bad_coefficient_layout(self):
        self.native['coef_x'] = []
        with self.assertRaises(ValueError):
            suffix(self.native, 10_000_000_000, 0.5, 0.)

    def test_invalid_duration(self):
        self.native['duration'][0] = float('nan')
        with self.assertRaises(ValueError):
            suffix(self.native, 10_000_000_000, 0.5, 0.)

    def test_subnanosecond_coverage_cannot_form_an_executable_path(self):
        self.native['duration'] = [0.1/1_000_000_000]
        for key in ('coef_x', 'coef_y', 'coef_z'):
            self.native[key] = self.native[key][:6]
        with self.assertRaises(ValueError):
            suffix(self.native, 10_000_000_000, 0.5, 0.)

    def test_hover_is_stop_intent(self):
        self.native['hover'] = True
        with self.assertRaises(ValueError):
            suffix(self.native, 10_000_000_000, 0.5, 0.)

    def test_stamp_roundtrip(self):
        value = 10_999_999_999
        self.assertEqual(stamp_ns(split_ns(value)), value)

    def test_yaw_across_pi_is_unwrapped(self):
        self.assertGreater(unwrap(-math.pi + 0.1, math.pi - 0.1), math.pi)

    def test_rolling_yaw_uses_future_reference(self):
        self.native['yaw'] = math.pi/2
        first = suffix(self.native, 10_000_000_000, 0.5, 0.0)
        reference = yaw_at(first, 10_000_000_000, 10_500_000_000)
        self.assertEqual(reference, 1.0)  # upstream 2rad/s at half a second
        second = suffix(self.native, 10_500_000_000, 0.5, reference)
        self.assertEqual(second[0]['yaw'], reference)

    def test_yaw_sampler_keeps_unwrapped_short_arc(self):
        rows = [{'time_ns': 0, 'yaw': math.pi-0.1},
                {'time_ns': 1_000_000_000, 'yaw': math.pi+0.1}]
        self.assertEqual(yaw_at(rows, 0, 500_000_000), math.pi)


if __name__ == '__main__':
    unittest.main()
