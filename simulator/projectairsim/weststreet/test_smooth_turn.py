import math
import unittest
from smooth_turn import rounded_path, profile, sample, candidate_from_baseline


class SmoothTurnTests(unittest.TestCase):
    def route(self):
        return rounded_path([(0,0),(0,40),(40,40),(40,0),(0,0)],4,.2)

    def test_circular_radius_and_tangency(self):
        points=rounded_path([(0,0),(0,20),(20,20)],4,.1)
        arc=[p for p in points if p[0]>1e-8 and p[1]<20-1e-8]
        self.assertGreater(len(arc),20)
        for x,y in arc:self.assertAlmostEqual(math.hypot(x-4,y-16),4)
        self.assertEqual(points[-1],(20,20))

    def test_refuses_radius_reduction(self):
        with self.assertRaises(ValueError):rounded_path([(0,0),(0,2),(2,2)],4)

    def test_speed_and_acceleration_bounds(self):
        p=profile(self.route())
        self.assertAlmostEqual(min(p['speeds_mps']),2.75)
        self.assertAlmostEqual(max(p['speeds_mps']),5.)
        self.assertEqual(p['speeds_mps'][0],p['speeds_mps'][-1])
        for i in range(len(p['times_s'])-1):
            dt=p['times_s'][i+1]-p['times_s'][i]
            self.assertGreater(dt,0)
            self.assertLessEqual(abs(p['speeds_mps'][i+1]-p['speeds_mps'][i])/dt,1.25+1e-8)

    def test_animation_phase_distance_and_sampling(self):
        p=profile(self.route())
        for t,d,phase in zip(p['times_s'],p['distance_m'],p['animation_phase_s']):
            self.assertAlmostEqual(sample(p,t)[1],d)
            self.assertAlmostEqual(phase,d/5.)
        self.assertEqual(sample(p,0)[0],sample(p,p['duration_s'])[0])

    def test_candidate_preserves_obstacle_suffix(self):
        original=[(4,-40),(4,64),(122,64),(122,-10),
                  (112,-9.25),(111.5,-9.2125),(110,-9.1),
                  (108,-9.2),(100,-10),(4,-40)]
        candidate=candidate_from_baseline(original)
        self.assertEqual(candidate[-5:],original[-5:])
        self.assertEqual(candidate[0],candidate[-1])
        self.assertGreater(profile(candidate)['duration_s'],0)

    def test_invalid_profile(self):
        with self.assertRaises(ValueError):profile([(0,0),(1,1)])
        with self.assertRaises(ValueError):profile([(0,0),(1,1),(0,0)],turn=6)


class ShiftedJunctionTest(unittest.TestCase):
    def test_initial_pose_and_far_suffix_unchanged(self):
        from smooth_turn import shifted_first_junction
        base=rounded_path([(4,-40),(4,64),(122,64),(122,-40),(4,-40)])
        shifted=shifted_first_junction(base)
        self.assertEqual(shifted[0],base[0]);self.assertEqual(shifted[-1],base[-1])
        self.assertEqual(shifted[-100:],base[-100:])
        turn=[(p,q)for p,q in zip(base,shifted)if 60<p[1]<64 and 4<p[0]<8]
        self.assertTrue(turn)
        for p,q in turn:
            self.assertAlmostEqual(p[0]-q[0],2.);self.assertEqual(p[1],q[1])
        self.assertAlmostEqual(max(profile(shifted)['speeds_mps']),5.)
        self.assertAlmostEqual(min(profile(shifted)['speeds_mps']),2.75)

class DelayedJunctionTest(unittest.TestCase):
    def test_first_fillet_translation_and_loop_preserved(self):
        from smooth_turn import shifted_first_junction, delayed_first_junction
        base=rounded_path([(4,-40),(4,64),(122,64),(122,-40),(4,-40)])
        delayed=delayed_first_junction(shifted_first_junction(base))
        self.assertEqual(delayed[0],base[0])
        self.assertEqual(delayed[-100:],base[-100:])
        turn=[(p,q) for p,q in zip(base,delayed) if 60<p[1]<64 and 4<p[0]<8]
        self.assertTrue(turn)
        for p,q in turn:
            self.assertAlmostEqual(q[0],p[0]-2)
            self.assertAlmostEqual(q[1],p[1]+4)
        plan=profile(delayed)
        self.assertAlmostEqual(min(plan['speeds_mps']),2.75)
        self.assertAlmostEqual(max(plan['speeds_mps']),5.)
        for i in range(len(plan['times_s'])-1):
            dt=plan['times_s'][i+1]-plan['times_s'][i]
            self.assertLessEqual(abs(plan['speeds_mps'][i+1]-plan['speeds_mps'][i])/dt,1.25+1e-8)

if __name__=='__main__':unittest.main()
