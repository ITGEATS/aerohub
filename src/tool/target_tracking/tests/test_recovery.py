"""Opt-in blind window preserves association limits and original sample time."""
import importlib.util
from pathlib import Path
import unittest
from uav_target_tracking.core import Tracker
root=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location('recovery_ekf',root/'deployment/airsim/demos/target_tracking/reference/yopo_tracker/YOPO/policy/target_ekf.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class RecoveryTest(unittest.TestCase):
    def seed(self,window=None):
        t=Tracker(module.EKF,measurement_stddev_m=1.,association_policy='soft_innovation_v1',max_prediction_time=window)
        for i in range(12):t.observe([[1,2,1]],1+i*.1,'map')
        self.assertFalse(t.ekf.lost)
        return t
    def test_default_expires_but_opt_in_recovers(self):
        old,new=self.seed(),self.seed(1.)
        stamp=new.observation_stamp
        for dt in (.1,.2,.3,.4,.5,.6,.7):
            old.observe([],stamp+dt,'map');new.observe([],stamp+dt,'map')
        self.assertFalse(old.snapshot(stamp+.75)[1])
        self.assertTrue(new.snapshot(stamp+.75)[1])
        self.assertEqual(new.observation_stamp,stamp)
        self.assertEqual(new.observe([[1,2,1]],stamp+.804,'map'),'tracked')
        self.assertTrue(new.snapshot(stamp+.85)[1])
        self.assertFalse(new.snapshot(stamp+1.805)[1])
    def test_larger_prediction_window_does_not_expand_soft_association(self):
        a,b=self.seed(),self.seed(1.)
        for t in (a,b):t.ekf.check_valid([[2,2,1]])
        self.assertEqual(a.ekf.update_audit['reach_m'],b.ekf.update_audit['reach_m'])
        self.assertEqual(b.ekf.association_blind_time,.5)
    def test_invalid_configuration(self):
        for value in (.1,2.1,float('nan')):
            with self.assertRaises(ValueError):Tracker(module.EKF,max_prediction_time=value)

if __name__=='__main__':unittest.main()
