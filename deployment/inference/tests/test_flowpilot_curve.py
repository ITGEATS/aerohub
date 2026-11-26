"""CPU-only regression for the FlowPilot planner-side curve handoff."""
import sys
import unittest
from pathlib import Path

import numpy as np

repo = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(repo / "src/tool/navigation/planner/flowpilot/ros2_adapter")]

from uav_flowpilot_planner.curve import Segment, state_derivatives, handoff_errors, brake_segment  # noqa: E402
from uav_flowpilot_planner.core import Timeline


class CurveTest(unittest.TestCase):
    def test_velocity_history_fit(self):
        times = np.linspace(-.2, 0., 5)
        acc = np.array([1., 2., 3.])
        jerk = np.array([4., 5., 6.])
        velocities = times[:, None] * acc + .5 * times[:, None] ** 2 * jerk
        fitted_acc, fitted_jerk = state_derivatives(times, velocities, 0.)
        np.testing.assert_allclose([fitted_acc, fitted_jerk], [acc, jerk], atol=1e-10)

    def test_velocity_history_rejects_degenerate_samples(self):
        with self.assertRaises(ValueError):
            state_derivatives([0., 0., 0.], np.zeros((3, 3)), 0.)

    def test_handoff_keeps_original_time_axis(self):
        controls = np.tile(np.array([1., 2., 3.]), (8, 1))
        segment = Segment(controls, np.zeros(3), np.eye(3), 10., 10.2, 0.4)
        self.assertEqual(segment.start, 10.2)
        self.assertAlmostEqual(segment.end, 10. + 47 / 30)
        self.assertEqual(segment.times[0], 0.)
        self.assertLess(segment.times[-1], 47 / 30)
        position, yaw = segment.sample(10.2)
        np.testing.assert_allclose(position[0], [1., 2., 3.], atol=1e-10)
        np.testing.assert_allclose(position[1:], 0., atol=1e-10)
        self.assertEqual(yaw, 0.4)

    def test_handoff_outside_sampled_window_rejected(self):
        controls = np.zeros((8, 3))
        with self.assertRaises(ValueError):
            Segment(controls, np.zeros(3), np.eye(3), 10., 12., 0.)

    def test_history_rejects_reversal_and_nonfinite_stamps(self):
        for times in ([-.1, -.05, -.08, 0.], [-.1, float('nan'), 0.]):
            with self.assertRaises(ValueError):
                state_derivatives(times, np.zeros((len(times), 3)), 0.)

    def test_yaw_rate_and_wrap(self):
        # Motion points just beyond -pi while the initial yaw is just below +pi.
        controls = np.arange(8)[:, None] * np.array([-1., -.1, 0.]) / 7
        segment = Segment(controls, np.zeros(3), np.eye(3), 10., 10.2, np.pi-.02, .3)
        rates = np.diff(segment.yaws) / np.diff(segment.times)
        self.assertTrue(np.all(np.abs(rates) <= .3 + 1e-10))
        self.assertAlmostEqual(segment.sample(segment.start)[1], np.pi-.02)
        timeline = Timeline()
        timeline.propose(segment)
        timeline.acknowledge(True)
        epoch = 10.42345
        pva, yaw = timeline.reference(epoch)
        expected = segment.execution_sample(epoch)
        np.testing.assert_array_equal(pva, expected[0])
        self.assertEqual(yaw, expected[1])

    def test_rejected_update_preserves_accepted_reference(self):
        controls = np.zeros((8, 3))
        old = Segment(controls, np.zeros(3), np.eye(3), 10., 10.2, .4)
        timeline = Timeline()
        timeline.propose(old)
        timeline.acknowledge(True)
        new = Segment(controls, [1., 0., 0.], np.eye(3), 10.1, 10.4, .4)
        errors = handoff_errors(timeline.reference(10.4), new)
        self.assertAlmostEqual(errors['position'], 1.)
        timeline.propose(new)
        timeline.acknowledge(False)
        self.assertIs(timeline.latest, old)
        self.assertEqual(timeline.revision, 1)

    def test_sample_times_remain_unique_after_nanosecond_rounding(self):
        segment = Segment(np.zeros((8, 3)), np.zeros(3), np.eye(3), 10., 10.2000000001, 0.)
        stamps = np.rint(segment.times * 1e9).astype(np.int64)
        self.assertEqual(stamps[0], 0)
        self.assertTrue(np.all(np.diff(stamps) > 0))

    def test_brake_segment_pins_reference_pva(self):
        # 2026-11-24 4090 flights: the behavior rejected braking stops whose
        # start dropped the plan's acceleration to zero (C2 replacement
        # gate, 0.91 and 1.84 m/s^2 measured). The brake must pin the full
        # reference PVA so the replacement is continuous by construction,
        # then bring the velocity to rest.
        pva = np.array([[1., 2., 3.], [.6, -.4, .2], [.9, .1, -.2]])
        curve, duration = brake_segment((pva, .3), 10.5, 1.0)
        np.testing.assert_allclose(curve.sample(curve.start)[0], pva, atol=1e-9)
        self.assertGreaterEqual(duration, .6)
        end = curve.sample(curve.end)[0]
        self.assertLess(np.linalg.norm(end[1]), .15)
        np.testing.assert_allclose(end[0], pva[0] + pva[1] * duration / 2, atol=.05)


if __name__ == "__main__":
    unittest.main()
