"""Coalesce independent yaw updates under the active execution task identity."""
import unittest
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock
from uav_planner_framework.follow_path_client import FleetFollowPathClient

class ContinuousYawClientTest(unittest.TestCase):
    def client(self):
        c=FleetFollowPathClient.__new__(FleetFollowPathClient)
        c.closed=c.failed=c.cancel_started=False;c.generation=0
        c.yaw_targets={};c.yaw_pending={};c.execution_tasks={'a':'task-uuid'}
        c.handles={'a':Mock()};c.results=set();c.yaw_clients={'a':Mock()}
        c.yaw_clients['a'].service_is_ready.return_value=True
        c.node=Mock();c.node.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        c._fail=Mock()
        return c

    def test_latest_yaw_does_not_wait_for_position_plan(self):
        c=self.client();one,two=Future(),Future()
        c.yaw_clients['a'].call_async.side_effect=[one,two]
        c.set_target_yaw('a',.2,11.,2)
        c.set_target_yaw('a',.4,11.,3)
        c.set_target_yaw('a',.6,11.,4)
        self.assertEqual(c.yaw_clients['a'].call_async.call_count,1)
        one.set_result(NS(accepted=True))
        request=c.yaw_clients['a'].call_async.call_args[0][0]
        self.assertEqual(request.task_id,'task-uuid');self.assertEqual(request.revision,4)
        self.assertEqual(request.target_yaw,.6)
        two.set_result(NS(accepted=True));c._fail.assert_not_called()

    def test_stopping_and_late_ack_cannot_restart(self):
        c=self.client();future=Future();c.yaw_clients['a'].call_async.return_value=future
        c.set_target_yaw('a',.2,11.,2)
        c.cancel_started=True;c.generation+=1
        c.set_target_yaw('a',.8,11.,3)
        future.set_result(NS(accepted=False,reason='inactive'))
        self.assertEqual(c.yaw_clients['a'].call_async.call_count,1)
        c._fail.assert_not_called()

    def test_ack_timeout_fails_closed(self):
        c=self.client();c.yaw_clients['a'].call_async.return_value=Future()
        c.pending=set();c.set_target_yaw('a',.2,11.,2)
        c.tick(11.1);c._fail.assert_called_once_with('CONTINUOUS_YAW_ACK_TIMEOUT')

if __name__=='__main__':unittest.main()
