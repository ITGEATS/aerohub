import importlib.util
from pathlib import Path
import unittest
import numpy as np

path=Path(__file__).resolve().parents[1]/'ros2_adapter/uav_target_tracking/measurement_filter.py'
spec=importlib.util.spec_from_file_location('measurement_filter',path)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class MeasurementFilterTests(unittest.TestCase):
    def test_constant_five_mps_has_no_moving_average_lag(self):
        f=m.ShortWindowFilter()
        for i in range(15):
            stamp=1+i*.1
            out=f.apply([[5*(stamp-1),0,1]],stamp)
            np.testing.assert_allclose(out,[[5*(stamp-1),0,1]],atol=1e-8)

    def test_isolated_spike_rejected_without_republishing_old_measurement(self):
        f=m.ShortWindowFilter()
        for i in range(5):f.apply([[i*.5,0,1]],1+i*.1)
        self.assertEqual(f.apply([[50,0,1]],1.5).shape,(0,3))
        np.testing.assert_allclose(f.apply([[3,0,1]],1.6),[[3,0,1]],atol=1e-8)

    def test_empty_and_multiple_detections_are_never_averaged_together(self):
        f=m.ShortWindowFilter()
        f.apply([[1,0,1]],1.)
        self.assertEqual(len(f.apply([],1.1)),0)
        points=[[1,0,1],[10,0,1]]
        np.testing.assert_array_equal(f.apply(points,1.2),points)
        np.testing.assert_array_equal(f.apply([[10,0,1]],1.3),[[10,0,1]])

    def test_transient_extra_candidate_does_not_erase_moving_history(self):
        f=m.ShortWindowFilter()
        # Recorded failure shape: extra candidate, then three descending noisy
        # ranges. Clearing history invented backwards motion and rejected all
        # four following observations. Ambiguous points must never be averaged.
        for t,x in [(35.436,12.0),(35.508,12.5),(35.577,13.1),
                    (35.718,12.17),(35.796,14.33)]:
            f.apply([[x,0,1]],t)
        history=[(t,p.copy()) for t,p in f.history]
        candidates=[[12.98,0,1],[21.79,-2.86,1.97]]
        np.testing.assert_array_equal(f.apply(candidates,35.871),candidates)
        self.assertEqual(len(f.history),len(history))
        for (t,p),(old_t,old_p) in zip(f.history,history):
            self.assertEqual(t,old_t)
            np.testing.assert_array_equal(p,old_p)
        for t,x in [(36.015,15.24),(36.09,14.69),(36.159,13.86),
                    (36.306,17.44),(36.375,15.83),(36.444,18.37),(36.591,17.46)]:
            self.assertEqual(f.apply([[x,0,1]],t).shape,(1,3))

    def test_long_gap_clears_history(self):
        f=m.ShortWindowFilter()
        for i in range(5):f.apply([[i*.5,0,1]],1+i*.1)
        np.testing.assert_array_equal(f.apply([[20,0,1]],2.),[[20,0,1]])

    def test_sparse_realistic_observations_reach_filter_not_permanent_warmup(self):
        f=m.ShortWindowFilter()
        f.apply([[0,0,1]],1.)
        f.apply([[1.5,0,1]],1.3)
        out=f.apply([[3,0,1]],1.6)
        self.assertEqual(f.last_status,'smoothed')
        np.testing.assert_allclose(out,[[3,0,1]],atol=1e-8)

    def test_bounded_jitter_is_smoothed_and_finite(self):
        f=m.ShortWindowFilter();raw=[];smooth=[]
        for i in range(40):
            truth=i*.5
            noise=[-.6,.5,.2,-.3,.1][i%5]
            out=f.apply([[truth+noise,0,1]],1+i*.1)
            if i>5:
                raw.append(noise**2);smooth.append((out[0,0]-truth)**2)
        self.assertLess(np.mean(smooth),np.mean(raw))
