import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tangent_policy import TangentReference

class TangentTests(unittest.TestCase):
    def test_low_speed_keeps_direction(self):
        p=TangentReference((-8,0),(0,0))
        _,a=p.update((8,0),(0,.1),.25)
        self.assertEqual(a['heading'],0.)
    def test_turn_and_reference_changes_are_bounded(self):
        p=TangentReference((-8,0),(0,0))
        last=p.point
        for _ in range(30):
            point,a=p.update((8,8),(-5,0),.25)
            self.assertLessEqual(abs(a['heading_step']),.2+1e-9)
            self.assertLessEqual(math.dist(last,point),1.75+1e-9)
            last=point
        self.assertAlmostEqual(abs(a['heading']),math.pi,places=5)
    def test_straight_reference_has_target_speed_after_transient(self):
        p=TangentReference((-8,0),(0,0))
        for i in range(1,41):
            point,_=p.update((8+5*i*.25,0),(5,0),.25)
        self.assertAlmostEqual(point[0],50+1.75)
        self.assertEqual(point[1],0.)
    def test_rejects_invalid_input(self):
        p=TangentReference((-8,0),(0,0))
        for dt in [0,-1,2]:
            with self.assertRaises(ValueError):p.update((8,0),(5,0),dt)
        with self.assertRaises(ValueError):p.update((float('nan'),0),(5,0),.25)


class LeadTests(unittest.TestCase):
    def test_lead_offsets_steady_reference_without_changing_speed(self):
        p=TangentReference((-8,0),(0,0),lead_seconds=1.)
        prev=None
        for i in range(1,41):
            point,_=p.update((8+5*i*.25,0),(5,0),.25)
            if i>30:self.assertAlmostEqual(point[0]-prev[0],1.25)
            prev=point
        self.assertAlmostEqual(point[0],55.)
    def test_bad_lead_rejected(self):
        for x in [-1,2,float('nan')]:
            with self.assertRaises(ValueError):TangentReference((-8,0),(0,0),lead_seconds=x)
