import importlib.util
from pathlib import Path
from types import SimpleNamespace as Obj
import unittest

spec = importlib.util.spec_from_file_location('follow_policy', Path(__file__).resolve().parents[1]/'follow_policy.py')
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


def stamp(value):
    return Obj(sec=int(value), nanosec=int((value-int(value))*1e9))


def target():
    return Obj(valid=True, status='tracked', reason='',
               header=Obj(frame_id='drone0/odom'), observation_stamp=stamp(10.),
               valid_until=stamp(10.8), position=Obj(x=20., y=10., z=1.))


class PolicyTests(unittest.TestCase):
    def destination(self, state, now=10.2):
        return policy.destination(state, now=now, frame='drone0/odom',
                                  altitude=3., offset=(-8., 0.))

    def test_follow_standoff_preserves_flight_altitude(self):
        self.assertEqual(policy.follow_offset((12., 10.), (20., 10.), 8.), (-8., 0.))
        self.assertEqual(self.destination(target()), (12., 10., 3.))

    def test_stale_or_future_observation_rejected(self):
        for now in (9.9, 10.7):
            with self.subTest(now=now), self.assertRaises(ValueError):
                self.destination(target(), now)

    def test_invalid_lost_or_frame_mismatch_rejected(self):
        for field, value in [('valid', False), ('status', 'lost'), ('status', 'confirm')]:
            state = target()
            setattr(state, field, value)
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.destination(state)
        state = target()
        state.header.frame_id = 'earth'
        with self.assertRaises(ValueError):
            self.destination(state)

    def test_expired_validity_and_nan_rejected(self):
        state = target()
        state.valid_until = stamp(10.1)
        with self.assertRaises(ValueError):
            self.destination(state)
        state = target()
        state.position.x = float('nan')
        with self.assertRaises(ValueError):
            self.destination(state)

    def test_bounded_prediction_allowed(self):
        state = target()
        state.status = 'predicted'
        self.assertEqual(self.destination(state), (12., 10., 3.))

    def test_initial_horizontal_overlap_rejected(self):
        with self.assertRaises(ValueError):
            policy.follow_offset((1., 1.), (1., 1.), 8.)
