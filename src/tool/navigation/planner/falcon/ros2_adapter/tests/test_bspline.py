import math
import types
import unittest

from uav_falcon_adapter.bspline import (
    InvalidSpline, Sample, continuity_deltas, exceeds_continuity_limits,
    sample_message)


def message(traj_id=1):
    points = [types.SimpleNamespace(x=float(index), y=2.0*index, z=1.0)
              for index in range(6)]
    return types.SimpleNamespace(
        traj_id=traj_id, order=3, pos_pts=points,
        knots=[float(index - 3) for index in range(10)],
        yaw_pts=[0.0] * 6, yaw_dt=1.0)


class BSplineTests(unittest.TestCase):
    def test_uniform_cubic_preserves_position_velocity_and_acceleration(self):
        samples = sample_message(message(), period=0.5)
        self.assertEqual(len(samples), 7)
        self.assertAlmostEqual(samples[0].position[0], 1.0)
        self.assertAlmostEqual(samples[-1].position[0], 4.0)
        for sample in samples:
            self.assertAlmostEqual(sample.velocity[0], 1.0)
            self.assertAlmostEqual(sample.velocity[1], 2.0)
            self.assertAlmostEqual(sample.acceleration[0], 0.0)
            self.assertAlmostEqual(sample.yaw, 0.0)

    def test_nonmonotonic_knots_are_rejected(self):
        value = message()
        value.knots[5] = -10.0
        with self.assertRaises(InvalidSpline):
            sample_message(value)

    def test_shorter_yaw_coverage_holds_terminal_heading(self):
        value = message()
        value.yaw_pts = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
        value.yaw_dt = 0.5
        samples = sample_message(value, period=0.5)
        self.assertAlmostEqual(samples[-1].yaw, samples[3].yaw)

    def test_longer_yaw_coverage_is_truncated_at_position_horizon(self):
        value = message()
        value.yaw_dt = 2.0
        samples = sample_message(value, period=0.5)
        self.assertAlmostEqual(samples[-1].time_from_start, 3.0)

    def test_nonfinite_control_point_is_rejected(self):
        value = message()
        value.pos_pts[2].x = math.nan
        with self.assertRaises(InvalidSpline):
            sample_message(value)

    def test_replacement_continuity_uses_original_execution_epoch(self):
        previous = sample_message(message(), period=0.5)
        replacement = tuple(Sample(
            sample.time_from_start,
            (sample.position[0] + 1.0, sample.position[1] + 2.0, sample.position[2]),
            sample.velocity, sample.acceleration, sample.yaw)
            for sample in previous[:-2])
        deltas = continuity_deltas(previous, 10.0, replacement, 11.0)
        self.assertTrue(all(value < 1e-9 for value in deltas))

    def test_discontinuous_replacement_is_detected(self):
        previous = sample_message(message(), period=0.5)
        replacement = list(previous)
        first = replacement[0]
        replacement[0] = Sample(
            first.time_from_start, (99.0, *first.position[1:]),
            first.velocity, first.acceleration, first.yaw)
        deltas = continuity_deltas(previous, 10.0, replacement, 10.0)
        self.assertGreater(deltas[0], 90.0)

    def test_replacement_time_gap_is_rejected(self):
        previous = sample_message(message(), period=0.5)
        with self.assertRaises(InvalidSpline):
            continuity_deltas(previous, 10.0, previous, 20.0)

    def test_continuity_limit_allows_only_numerical_noise(self):
        limits = (0.25, 0.5, 1.4, 0.2)
        self.assertFalse(exceeds_continuity_limits(
            (0.2505, 0.5005, 1.4005, 0.2005), limits))
        self.assertTrue(exceeds_continuity_limits(
            (0.25, 0.502, 1.4, 0.2), limits))


if __name__ == "__main__":
    unittest.main()
