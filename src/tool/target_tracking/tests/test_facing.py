from concurrent.futures import Future
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_adapter'))
from uav_target_tracking.facing import FacingError, FacingSession, facing_yaw


def done(value):
    future = Future()
    future.set_result(value)
    return future


class Handle:
    def __init__(self, accepted=True, stopped=True):
        self.accepted, self.stopped = accepted, stopped
        self.result = Future()
        self.cancels = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        if not self.result.done():
            self.result.set_result(NS(result=NS(stop_confirmed=self.stopped,code='CANCELED',reason='canceled')))
        return done(NS())


class Transport:
    def __init__(self):
        self.handle = Handle()
        self.calls = []
        self.available = True
        self.submission = done(self.handle)
        self.accept_update = True

    def ready(self):
        return self.available

    def start(self, *args):
        self.calls.append(('start',args))
        return self.submission

    def update(self, *args):
        self.calls.append(('update',args))
        return done(NS(accepted=self.accept_update,target_revision=args[1],reason='rejected'))


class FacingTests(unittest.TestCase):
    def test_yaw_quadrants_and_invalid_targets(self):
        self.assertAlmostEqual(facing_yaw((0,0,3),(1,0,0)),0.)
        self.assertAlmostEqual(facing_yaw((0,0,3),(0,1,0)),math.pi/2)
        self.assertAlmostEqual(abs(facing_yaw((0,0,3),(-1,0,0))),math.pi)
        for target in ((0,0,8),(float('nan'),1,0)):
            with self.assertRaises(FacingError):
                facing_yaw((0,0,3),target)

    def test_no_submission_means_no_motion_to_stop(self):
        transport = Transport()
        self.assertTrue(FacingSession(transport).close())
        self.assertEqual(transport.calls,[])

    def test_position_is_frozen_and_yaw_updates_are_serial(self):
        transport = Transport()
        clock = [1.]
        session = FacingSession(transport,clock=lambda:clock[0])
        session.step((1,2,3),'earth',0.,10.)
        clock[0] += .21
        session.step((1.1,2.1,3.1),'earth',1.,9.)
        self.assertEqual(transport.calls[-1][1][2],(1,2,3))
        self.assertEqual(transport.calls[-1][1][-1],1.)
        self.assertEqual(session.revision,2)
        clock[0] += .21
        session.step((1,2,3),'earth',1.5,9.)
        self.assertEqual(session.revision,3)
        self.assertTrue(session.close())
        self.assertEqual(transport.handle.cancels,1)

    def test_unavailable_navigation_does_not_fall_back(self):
        transport = Transport()
        transport.available = False
        session = FacingSession(transport)
        with self.assertRaisesRegex(FacingError,'unavailable'):
            session.step((0,0,3),'earth',0.,10.)
        self.assertEqual(transport.calls,[])

    def test_unsupported_yaw_is_propagated(self):
        transport = Transport()
        session = FacingSession(transport)
        session.step((0,0,3),'earth',0.,10.)
        transport.handle.result.set_result(NS(result=NS(code='UNSUPPORTED',reason='yaw unsupported',stop_confirmed=True)))
        with self.assertRaises(FacingError) as raised:
            session.step((0,0,3),'earth',0.,10.)
        self.assertEqual(raised.exception.code,'NAVIGATION_UNSUPPORTED')
        self.assertTrue(session.close())

    def test_busy_goal_rejection_has_no_owned_motion(self):
        transport = Transport()
        transport.handle.accepted = False
        session = FacingSession(transport)
        session.step((0,0,3),'earth',0.,10.)
        with self.assertRaises(FacingError):
            session.step((0,0,3),'earth',0.,10.)
        self.assertTrue(session.close())
        self.assertEqual(transport.handle.cancels,0)

    def test_update_failure_requires_cancel_and_stop(self):
        transport = Transport()
        transport.accept_update = False
        clock = [1.]
        session = FacingSession(transport,clock=lambda:clock[0])
        session.step((0,0,3),'earth',0.,10.)
        clock[0] += .3
        session.step((0,0,3),'earth',1.,9.)
        with self.assertRaises(FacingError):
            session.step((0,0,3),'earth',1.,9.)
        self.assertTrue(session.close())
        self.assertEqual(transport.handle.cancels,1)

    def test_unknown_submission_cancels_late_but_does_not_claim_stop(self):
        transport = Transport()
        transport.submission = Future()
        session = FacingSession(transport,cleanup_timeout=.001)
        session.step((0,0,3),'earth',0.,10.)
        self.assertFalse(session.close())
        transport.submission.set_result(transport.handle)
        self.assertEqual(transport.handle.cancels,1)

    def test_unconfirmed_stop_remains_failure(self):
        transport = Transport()
        transport.handle.stopped = False
        session = FacingSession(transport)
        session.step((0,0,3),'earth',0.,10.)
        self.assertFalse(session.close())

    def test_frame_change_rejected_before_update(self):
        transport = Transport()
        session = FacingSession(transport)
        session.step((0,0,3),'earth',0.,10.)
        with self.assertRaises(FacingError):
            session.step((0,0,3),'other',0.,10.)
        self.assertEqual(len(transport.calls),1)
        self.assertTrue(session.close())


if __name__=='__main__':
    unittest.main()
