"""Recorded final-coverage gap must hand off, never modify an expired Action."""
import time
import unittest
from unittest.mock import Mock
from uav_super_adapter.node import SuperAdapter
from uav_planner_interfaces.msg import TimedTrajectorySegment,VehicleTrajectory,TimedTrajectoryPoint
from uav_planner_framework.ros_conversions import seconds_to_time


class DispatchTests(unittest.TestCase):
    def node(self,final=True,managed=True):
        n=SuperAdapter.__new__(SuperAdapter)
        n.now_seconds=lambda:35.181
        n.enabled=True;n.task='recorded';n.error='';n.vehicle='drone0'
        n.flags={};n.late_updates=0;n.last_wall=time.monotonic();n.started=time.monotonic()
        n.reference_ready=True;n.publisher=Mock();n.get_logger=lambda:Mock()
        n.client=Mock(revision=9,complete=False,ready=True,pending=set(),failed=False,cancel_started=False)
        n.client.settling_vehicles=set()
        n.submitted_segment=dict(end_ns=35153291604,final=final)
        n.last_curve=dict(source_end=35.153291604490114,is_final=final)
        n.navigation=Mock()if managed else None
        if managed:n.navigation.handoff_final_coverage.return_value=True
        segment=TimedTrajectorySegment();seconds_to_time(35.281,segment.valid_from)
        point=TimedTrajectoryPoint();seconds_to_time(.96777206,point.time_from_start)
        segment.vehicle_trajectories=[VehicleTrajectory(points=[TimedTrajectoryPoint(),point])]
        segment.is_final=True
        n.convert=lambda _:segment
        n.queued=Mock();seconds_to_time(35.163,n.queued.header.stamp)
        n.fail=Mock()
        return n

    def test_recorded_gap_reacquires_after_stop_instead_of_modifying(self):
        n=self.node()
        SuperAdapter.tick(n)
        n.client.submit.assert_not_called()
        n.navigation.handoff_final_coverage.assert_called_once()
        n.fail.assert_not_called()

    def test_completed_execution_with_queued_plan_routes_to_new_action(self):
        n=self.node();n.client.complete=True
        SuperAdapter.tick(n)
        n.client.submit.assert_not_called()
        n.navigation.handoff_final_coverage.assert_called_once_with(completed=True)

    def test_nonfinal_gap_is_not_silently_restarted(self):
        n=self.node(final=False)
        SuperAdapter.tick(n)
        n.client.submit.assert_not_called()
        self.assertEqual(n.fail.call_args.args[0],'TRAJECTORY_COVERAGE_GAP')

    def test_unmanaged_final_gap_still_fails_closed(self):
        n=self.node(managed=False)
        SuperAdapter.tick(n)
        n.client.submit.assert_not_called()
        self.assertEqual(n.fail.call_args.args[0],'TRAJECTORY_COVERAGE_GAP')

    def test_overlapping_update_keeps_original_time(self):
        n=self.node();n.submitted_segment['end_ns']=36000000000
        SuperAdapter.tick(n)
        n.client.submit.assert_called_once()
        segment=n.client.submit.call_args.args[0]
        self.assertAlmostEqual(segment.valid_from.sec+segment.valid_from.nanosec/1e9,35.281)
        n.navigation.handoff_final_coverage.assert_not_called()


class HandoffTests(unittest.TestCase):
    def nav(self):
        from uav_super_adapter.navigation import SuperNavigation
        from uav_planner_interfaces.action import NavigateToPose
        from uav_planner_interfaces.srv import UpdateNavigationTarget
        n=SuperNavigation.__new__(SuperNavigation)
        n.node=Mock();n.node.now_seconds.return_value=10.;n.node.goal=[0.,0.,3.]
        n.reserved=True;n.outcome=None;n.stop_started=None;n.resume_after_stop=False
        n.holding=False;n.started=1.;n.timeout=100.;n.last_target=9.9;n.target_revision=1
        n.finished=Mock();n.finished.done.return_value=False
        request=UpdateNavigationTarget.Request();request.target_revision=2
        request.target.pose.position.x=2.;request.target.pose.position.z=3.
        n.handle=Mock(is_cancel_requested=False);n.handle.goal_id=request.goal_id
        n.handle.request=NavigateToPose.Goal();n.handle.request.arrival_behavior=n.handle.request.HOLD
        n.validate=Mock(return_value=None);n.publish_target=Mock();n.restart_execution=Mock()
        return n,request,UpdateNavigationTarget.Response()

    def test_handoff_stops_without_reacquiring_early(self):
        n,_,_=self.nav()
        self.assertTrue(n.handoff_final_coverage())
        self.assertTrue(n.resume_after_stop)
        self.assertIsNotNone(n.stop_started)
        self.assertFalse(n.node.enable.call_args.args[0].data)
        n.restart_execution.assert_not_called()

    def test_latest_target_is_buffered_during_planned_stop(self):
        n,req,res=self.nav();n.handoff_final_coverage()
        self.assertTrue(n.update_target(req,res).accepted)
        self.assertEqual(n.node.goal,[2.,0.,3.])
        n.publish_target.assert_not_called()
        n.restart_execution.assert_not_called()

    def test_new_target_after_completed_execution_does_not_modify(self):
        n,req,res=self.nav();n.node.client.complete=True
        self.assertTrue(n.update_target(req,res).accepted)
        self.assertTrue(n.resume_after_stop)
        n.publish_target.assert_not_called()
        self.assertFalse(n.node.enable.call_args.args[0].data)

    def test_unchanged_target_after_completion_is_only_renewed(self):
        n,req,res=self.nav();n.node.goal=[2.,0.,3.];n.node.client.complete=True
        self.assertTrue(n.update_target(req,res).accepted)
        n.node.enable.assert_not_called()
        n.publish_target.assert_not_called()

    def test_cancel_is_still_rejected_during_handoff(self):
        n,req,res=self.nav();n.handoff_final_coverage();n.handle.is_cancel_requested=True
        self.assertFalse(n.update_target(req,res).accepted)
        self.assertEqual(res.code,'STOPPING')


if __name__=='__main__':unittest.main()
