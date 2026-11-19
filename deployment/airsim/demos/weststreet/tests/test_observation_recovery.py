from pathlib import Path
from types import SimpleNamespace as S
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from observation_recovery import ObservationRecovery


def stamp(t):
    return S(sec=int(t),nanosec=round((t-int(t))*1e9))


def state(observed=10.,valid=True):
    return S(header=S(frame_id='drone0/odom'),generation=3,
        observation_stamp=stamp(observed),valid_until=stamp(observed+1),
        status='tracked' if valid else 'lost',valid=valid,
        position=S(x=8.,y=0.,z=1.),velocity=S(x=5.,y=0.,z=0.))


class RecoveryTests(unittest.TestCase):
    def new(self):return ObservationRecovery('drone0/odom',3,10.,2.)
    def test_expired_observation_waits_then_new_fresh_capture_resumes(self):
        p=self.new();self.assertFalse(p.update(state(),10.2,1)[0])
        old=state(valid=False)
        self.assertTrue(p.update(old,11.01,2)[0]);self.assertFalse(old.valid)
        waiting,elapsed=p.update(state(11.1),11.3,2.3)
        self.assertFalse(waiting);self.assertAlmostEqual(elapsed,.3)
    def test_repeated_lost_feedback_does_not_renew_grace(self):
        p=self.new();p.update(state(valid=False),11.01,2)
        self.assertTrue(p.update(state(valid=False),11.5,2.5)[0])
        with self.assertRaises(TimeoutError):p.update(state(valid=False),13.1,4.1)
    def test_late_recovery_does_not_override_timeout(self):
        p=self.new();p.update(state(valid=False),11.01,2)
        with self.assertRaises(TimeoutError):p.update(state(13.0),13.1,4.1)
    def test_old_or_aged_capture_does_not_count_as_recovery(self):
        p=self.new();p.update(state(valid=False),11.01,2)
        self.assertTrue(p.update(state(10.5),11.1,2.1)[0])
        self.assertFalse(p.update(state(11.1),11.3,2.3)[0])
    def test_frame_generation_and_clock_changes_remain_failures(self):
        for change in ['frame','generation','clock','future']:
            p=self.new();p.update(state(),10.2,1)
            s=state(valid=False);now=11.1
            if change=='frame':s.header.frame_id='other'
            if change=='generation':s.generation=4
            if change=='clock':now=9.9
            if change=='future':s.observation_stamp=stamp(12.)
            with self.subTest(change=change),self.assertRaises(ValueError):p.update(s,now,2)
    def test_nonfinite_valid_pose_is_not_treated_as_missing(self):
        p=self.new();s=state();s.position.x=float('nan')
        with self.assertRaises(ValueError):p.update(s,10.2,1)
