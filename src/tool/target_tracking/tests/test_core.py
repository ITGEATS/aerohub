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

    def test_known_height_noise_preserves_real_running_trace(self):
        import json
        rows = json.loads((Path(__file__).parent/'fixtures/weststreet_height_positions.json').read_text())['observations']
        default = Tracker(m.EKF)
        height = Tracker(m.EKF, measurement_stddev_m=1.)
        confirmed = False
        original_failed = False
        for row in rows:
            default.observe(row['points'], row['stamp'], 'map')
            height.observe(row['points'], row['stamp'], 'map')
            valid = height.snapshot(row['returned'])[1]
            if confirmed:
                self.assertTrue(valid, row)
                original_failed |= not default.snapshot(row['returned'])[1]
            confirmed |= valid
        self.assertTrue(confirmed and original_failed)
        self.assertEqual(height.ekf.max_blind_time, .5)
        self.assertEqual(height.ekf.distance_threshold, 1.)
        self.assertFalse(height.snapshot(rows[-1]['stamp']+1.)[1])

    def test_measurement_noise_validation(self):
        for value in (0., -1., float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                Tracker(m.EKF, measurement_stddev_m=value)

    def test_innovation_association_independent_running_trace(self):
        import json
        rows=json.loads((Path(__file__).parent/'fixtures/weststreet_height_positions_11.json').read_text())['observations']
        tracker=Tracker(m.EKF, measurement_stddev_m=1., association_policy='innovation_covariance_v1')
        confirmed=False
        for row in rows:
            tracker.observe(row['points'],row['stamp'],'map')
            valid=tracker.snapshot(row['returned'])[1]
            if confirmed:
                self.assertTrue(valid,row)
            confirmed |= valid
        self.assertTrue(confirmed)
        self.assertFalse(tracker.snapshot(rows[-1]['stamp']+1.)[1])
        ids,_=tracker.ekf.check_valid(np.array([tracker.ekf.x[:3]+[3.1,0.,0.]]))
        self.assertEqual(len(ids),0)  # covariance never overrides hard3m bound

    def test_association_configuration_requires_source(self):
        for policy in ('bad', 'innovation_covariance_v1'):
            with self.assertRaises(ValueError):
                Tracker(m.EKF, association_policy=policy)

    def test_prefilter_rejected_jump_never_refreshes_observation(self):
        t=Tracker(m.EKF, position_filter='robust_linear_v1')
        for i in range(8):t.observe([[1+i*.1,0,1]],1+i*.1,'map')
        accepted=t.observation_stamp
        t.observe([[50,0,1]],1.8,'map')
        self.assertEqual(t.measurement_audit['status'],'jump_rejected')
        self.assertEqual(t.measurement_audit['filtered'],[])
        self.assertEqual(t.observation_stamp,accepted)
        self.assertFalse(t.snapshot(accepted+.501)[1])

    def test_soft_update_preserves_recorded_noisy_motion(self):
        import json
        for name in ('startup','range'):
            rows=json.loads((Path(__file__).parent/('fixtures/weststreet_soft_'+name+'.json')).read_text())['observations']
            t=Tracker(m.EKF,measurement_stddev_m=1.,association_policy='soft_innovation_v1')
            confirmed=False
            for row in rows:
                t.observe(row['points'],row['stamp'],'map')
                if confirmed:self.assertTrue(t.snapshot(row['returned'])[1],(name,row))
                confirmed |= t.snapshot(row['returned'])[1]
                self.assertGreaterEqual(np.linalg.eigvalsh(t.ekf.Sigma).min(),-1e-9)
            self.assertTrue(confirmed)
            self.assertFalse(t.snapshot(rows[-1]['stamp']+.501)[1])

    def test_soft_update_bounds_spike_and_recovers_without_gate_lock(self):
        t=Tracker(m.EKF,measurement_stddev_m=1.,association_policy='soft_innovation_v1')
        for i in range(10):t.observe([[1.,0.,1.]],1+i*.1,'map')
        before=t.ekf.x.copy()
        t.observe([[5.,0.,1.]],2.,'map')
        self.assertEqual(t.observation_stamp,2.)
        self.assertLessEqual(np.linalg.norm(t.ekf.x[:3]-before[:3]),1.+1e-8)
        self.assertLess(t.measurement_audit['association']['position_correction_m'],1.+1e-8)
        for i in range(1,10):t.observe([[5.,0.,1.]],2+i*.1,'map')
        self.assertGreater(t.ekf.x[0],3.)
        accepted=t.observation_stamp
        t.observe([[50.,0.,1.]],3.,'map')
        self.assertEqual(t.observation_stamp,accepted)
        t.observe([],3.6,'map')
        self.assertFalse(t.snapshot(3.6)[1])

    def test_soft_update_does_not_choose_between_equal_candidates(self):
        t=Tracker(m.EKF,measurement_stddev_m=1.,association_policy='soft_innovation_v1')
        for i in range(10):t.observe([[1.,0.,1.]],1+i*.1,'map')
        accepted=t.observation_stamp
        t.observe([[1.,.1,1.],[1.,-.1,1.]],2.,'map')
        self.assertEqual(t.observation_stamp,accepted)

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
