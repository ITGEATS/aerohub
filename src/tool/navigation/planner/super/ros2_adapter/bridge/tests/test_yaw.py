"""Optional target yaw validation and original-epoch polynomial conversion."""
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from uav_super_adapter.polynomial import yaw_polynomial
from uav_super_adapter.navigation import SuperNavigation
from uav_super_adapter.node import SuperAdapter
from uav_planner_interfaces.action import NavigateToPose
from mars_quadrotor_msgs.msg import PolynomialTrajectory

class YawTest(unittest.TestCase):
    def message(self):
        m=PolynomialTrajectory()
        m.start_wt_pos=100.;m.time_pos=[1.];m.order_pos=1;m.piece_num_pos=1
        m.coef_pos_x=[1.,0.];m.coef_pos_y=[0.,0.];m.coef_pos_z=[0.,2.]
        m.start_wt_yaw=99.5;m.time_yaw=[1.5];m.order_yaw=1;m.piece_num_yaw=1
        m.coef_yaw=[.5,0.]
        return m

    def test_yaw_epoch_and_missing_coverage(self):
        m=self.message();c=yaw_polynomial(m,100.,101.)
        self.assertAlmostEqual(c.sample(100.)[0][0],.25)
        m.start_wt_yaw=100.5
        with self.assertRaises(ValueError):yaw_polynomial(m,100.,101.)
        m=self.message();m.coef_yaw=[math.nan,0.]
        with self.assertRaises(ValueError):yaw_polynomial(m,100.,101.)

    def test_opt_in_and_quaternion(self):
        n=Mock(frame='drone0/odom');n.get_parameter.return_value.value=False
        nav=SuperNavigation.__new__(SuperNavigation);nav.node=n
        r=NavigateToPose.Goal();r.target.header.frame_id=n.frame;r.orientation_mode=r.TARGET_YAW
        r.target.pose.orientation.w=1.
        self.assertEqual(nav.validate(r)[0],'UNSUPPORTED')
        n.get_parameter.return_value.value=True;r.target.pose.orientation.w=0.
        self.assertEqual(nav.validate(r)[0],'INVALID_GOAL')
        r.target.pose.orientation.x=1.
        self.assertEqual(nav.validate(r)[0],'UNSUPPORTED')

    def test_publish_preserves_requested_yaw_only_in_target_mode(self):
        from geometry_msgs.msg import PoseStamped
        nav=SuperNavigation.__new__(SuperNavigation)
        request=NavigateToPose.Goal();request.orientation_mode=request.TARGET_YAW
        nav.handle=NS(request=request);nav.target=PoseStamped()
        nav.target.pose.orientation.z=math.sin(.5);nav.target.pose.orientation.w=math.cos(.5)
        nav.odom=NS(pose=NS(pose=NS(orientation=PoseStamped().pose.orientation)))
        nav.node=Mock();nav.publisher=Mock()
        from builtin_interfaces.msg import Time
        nav.node.get_clock.return_value.now.return_value.to_msg.return_value=Time()
        nav.publish_target()
        self.assertAlmostEqual(nav.publisher.publish.call_args[0][0].pose.orientation.z,math.sin(.5))
        request.orientation_mode=request.KEEP_YAW
        nav.publish_target()
        self.assertEqual(nav.publisher.publish.call_args[0][0].pose.orientation.z,0.)

    def test_convert_preserves_yaw_and_default_position_only(self):
        n=SuperAdapter.__new__(SuperAdapter)
        n.now_seconds=lambda:100.;n.get_parameter=lambda key:NS(value={'lead_time':.1,'target_yaw_enabled':True,'goal_tolerance':1.5}[key])
        n.client=NS(revision=0);n.task='test';n.goal=[1.,0.,2.];n.vehicle='drone0';n.navigation_tolerance=1.5
        request=NavigateToPose.Goal();request.orientation_mode=request.TARGET_YAW
        n.navigation=NS(reserved=True,handle=NS(request=request))
        m=self.message();s=n.convert(m);path=s.vehicle_trajectories[0]
        self.assertEqual(path.valid_fields,7)
        self.assertAlmostEqual(path.points[0].yaw,math.pi/2-.3)
        request.orientation_mode=request.UNCONSTRAINED
        m.coef_yaw=[] # No yaw needed by legacy mode.
        self.assertEqual(n.convert(m).vehicle_trajectories[0].valid_fields,3)

if __name__=='__main__':unittest.main()
