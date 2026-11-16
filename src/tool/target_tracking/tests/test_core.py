import importlib.util
import unittest
from pathlib import Path
import numpy as np
from uav_target_tracking.core import Tracker

ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location('original', ROOT / 'deployment/airsim/demos/target_tracking/reference/yopo_tracker/YOPO/policy/target_ekf.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


class CoreTests(unittest.TestCase):
    def test_fixed_dt_equivalence(self):
        original, wrapped = m.EKF(), Tracker(m.EKF)
        rng = np.random.default_rng(123)
        for i in range(200):
            points = np.empty((0, 3)) if i % 35 > 23 else np.array([[i * .02, 2, 1], [15, 20, 30]]) + rng.normal(0, .01, (2, 3))
            ids, pos, confirmed = original.predict_and_update(points)
            status = wrapped.observe(points, 1 + .1 * i, 'map')
            np.testing.assert_allclose(wrapped.ekf.x, original.x, atol=1e-10, rtol=1e-9)
            np.testing.assert_allclose(wrapped.ekf.Sigma, original.Sigma, atol=1e-10, rtol=1e-9)
            self.assertEqual(wrapped.ekf.lost, original.lost)
            self.assertEqual(status in ('tracked', 'predicted'), confirmed)
            if confirmed:
                self.assertEqual(status == 'tracked', bool(ids.size))

    def test_irregular_dt(self):
        t = Tracker(m.EKF)
        t.observe([[2, 3, 1]], 1, 'map')
        t.observe([[2, 3, 1]], 1.37, 'map')
        np.testing.assert_allclose(t.ekf.A[:3, 3:], np.eye(3) * .37)
        np.testing.assert_allclose(t.ekf.B[:3], np.eye(3) * .37**2/2)
        np.testing.assert_allclose(t.ekf.B[3:], np.eye(3) * .37)

    def test_reject_without_mutation(self):
        t = Tracker(m.EKF); t.observe([[1, 2, 3]], 1, 'map')
        before = t.ekf.x.copy()
        for stamp, frame, points in [(1, 'map', []), (.5, 'map', []), (2, 'odom', []), (2, 'map', [[float('nan'), 0, 0]])]:
            with self.assertRaises(ValueError): t.observe(points, stamp, frame)
            np.testing.assert_array_equal(t.ekf.x, before)

    def test_confirmation_prediction_loss(self):
        t = Tracker(m.EKF)
        self.assertEqual(t.snapshot(1)[0], 'unavailable')
        for i in range(10): status = t.observe([[1, 2, 3]], 1 + i * .1, 'map')
        self.assertEqual(status, 'tracked')
        self.assertTrue(t.snapshot(2)[1])
        self.assertEqual(t.snapshot(3)[0], 'lost')
        self.assertFalse(t.snapshot(3)[1])
        for i in range(8): t.observe([], 2 + i * .1, 'map')
        self.assertTrue(t.ekf.lost)
        for i in range(10): status = t.observe([[1, 2, 3]], 3 + i * .1, 'map')
        self.assertEqual(status, 'tracked')

    def test_feedback_prediction_does_not_advance_filter(self):
        t = Tracker(m.EKF)
        for i in range(10): t.observe([[1+i*.02, 2, 3]], 1+i*.1, 'map')
        x, cov = t.ekf.x.copy(), t.ekf.Sigma.copy()
        status, valid, predicted, _ = t.snapshot(2.)
        self.assertTrue(valid)
        self.assertEqual(status, 'predicted')
        np.testing.assert_allclose(predicted[:3], x[:3] + .1*x[3:])
        np.testing.assert_array_equal(t.ekf.x, x)
        np.testing.assert_array_equal(t.ekf.Sigma, cov)
        self.assertFalse(t.snapshot(1.)[1])

    def test_invalid_stamp_and_shape(self):
        for stamp, frame, points in [(float('nan'), 'map', []), (float('inf'), 'map', []),
                                     (0, 'map', []), (1, '', []), (1, 'map', [[1,2,3,4]])]:
            with self.assertRaises(ValueError): Tracker(m.EKF).observe(points, stamp, frame)

if __name__ == '__main__': unittest.main()
