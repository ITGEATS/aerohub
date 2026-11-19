import math
import unittest
from uav_super_adapter.polynomial import Polynomial


class CommittedPolynomialTest(unittest.TestCase):
    def test_absolute_epoch_pva_and_piece_boundary(self):
        # p=t^3, then p=(1+t)^3; the second committed piece is retained.
        x=[0,0,0,0,1,0,0,0]+[0,0,0,0,1,3,3,1]
        curve=Polynomial(100,[1,1],[x,[0]*16,[0]*16],7)
        for t in (0,.5,1,1.5,2):
            p,v,a=curve.sample(100+t)
            self.assertAlmostEqual(p[0],t**3)
            self.assertAlmostEqual(v[0],3*t*t)
            self.assertAlmostEqual(a[0],6*t)
        times=curve.times(100.15)
        self.assertEqual(times[0],100.15)
        self.assertIn(101,times)
        self.assertEqual(times[-1],102)

    def test_stopped_backup_suffix_is_not_discarded(self):
        # Primary moves to x=1, committed backup decelerates to x=1.5.
        curve=Polynomial(100,[1,1],[[0,0,1,0,0,-.5,1,1],[0]*8,[0]*8],3)
        self.assertEqual(curve.sample(102)[0],[1.5,0,0])
        self.assertEqual(curve.sample(102)[1],[0,0,0])
        self.assertEqual(curve.times(100.1)[-1],102)

    def test_recorded_course_boundaries_have_distinct_ros_times(self):
        # 2026-11-16 course plan: boundary and regular sample differed only
        # by floating point rounding, then both serialized to 120000000 ns.
        durations = [.054999999999999716, .09599999999999653,
                     .09900000000000375, .18424204630889485,
                     1.7002562183868806, 1.7002562183868806]
        curve = Polynomial(43.011, durations, [[0]*48]*3, 7)
        start = 43.141
        times = curve.times(start)
        nanos = [round((t-start)*1e9) for t in times]
        self.assertTrue(all(b > a for a,b in zip(nanos,nanos[1:])))
        self.assertEqual(times[0], start)
        self.assertEqual(times[-1], curve.end)
        for offset in curve.ends:
            boundary = curve.start+offset
            if boundary > start:
                self.assertIn(boundary, times)

    def test_rejects_malformed_and_never_extrapolates(self):
        for args in [(0,[1],[[0]*8]*3,7),(100,[0],[[0]*8]*3,7),
                     (100,[1],[[math.nan]*8]*3,7),(100,[1],[[0]*7]*3,7)]:
            with self.assertRaises(ValueError):Polynomial(*args)
        curve=Polynomial(100,[1],[[0]*8]*3,7)
        with self.assertRaises(ValueError):curve.sample(99)
        with self.assertRaises(ValueError):curve.sample(102)
        with self.assertRaises(ValueError):curve.times(100.99)

if __name__=='__main__':unittest.main()
