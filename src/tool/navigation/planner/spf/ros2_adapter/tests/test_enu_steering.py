"""Independent FLU/ENU steering invariants; upstream angles are clockwise/right-positive."""
import math
import unittest

from uav_spf_planner.projector import ActionPoint, project
from uav_spf_planner.trajectory import build_revision


class EnuSteering(unittest.TestCase):
    def plan(self, right, yaw=0.):
        return build_revision(ActionPoint(dx=right,dy=1.,dz=0.),0.,0.,1.5,yaw,1.,30.,2.)

    def test_right_pixel_projects_right_then_turns_clockwise_in_enu(self):
        action=project([500,1000],5,50.,50.,50.,50.,100,100,False)
        self.assertGreater(action.dx,0.)
        plan=build_revision(action,0.,0.,1.5,0.,1.,30.,2.)
        self.assertAlmostEqual(plan['points'][-1]['yaw'],-math.pi/4)
        self.assertLess(plan['points'][-1]['y'],0.)

    def test_left_target_turns_counterclockwise_in_enu(self):
        plan=self.plan(-1.)
        self.assertAlmostEqual(plan['points'][-1]['yaw'],math.pi/4)
        self.assertGreater(plan['points'][-1]['y'],0.)

    def test_heading_north_right_target_moves_east(self):
        plan=self.plan(1.,math.pi/2)
        self.assertAlmostEqual(plan['points'][-1]['yaw'],math.pi/4)
        self.assertGreater(plan['points'][-1]['x'],0.)


if __name__=='__main__':
    unittest.main(verbosity=2)
