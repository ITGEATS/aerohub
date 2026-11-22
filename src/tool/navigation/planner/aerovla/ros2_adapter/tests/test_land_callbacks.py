"""Land result/cancel races and timeout boundary; requires ROS message classes."""
import unittest
from concurrent.futures import Future
from unittest.mock import Mock

try:
    from action_msgs.msg import GoalStatus
    from uav_aerovla_planner.goto_node import AerovlaGoToNode
except ImportError:
    AerovlaGoToNode = None


@unittest.skipIf(AerovlaGoToNode is None, 'requires ROS')
class LandCallbacksTest(unittest.TestCase):
    def test_native_success_requires_successful_status_and_payload(self):
        for status, success in ((GoalStatus.STATUS_ABORTED, True), (GoalStatus.STATUS_SUCCEEDED, False)):
            node = Mock(phase='LANDING')
            future = Future()
            future.set_result(Mock(status=status, result=Mock(land_success=success)))
            AerovlaGoToNode.land_finished(node, future)
            node.fail.assert_called_once_with('LAND_FAILED')
            node.publish_status.assert_not_called()

    def test_late_land_success_during_cancel_only_confirms_terminal_action(self):
        node = Mock(phase='STOPPING', state='STOPPED', land_success=False)
        future = Future()
        future.set_result(Mock(status=GoalStatus.STATUS_SUCCEEDED, result=Mock(land_success=True)))
        AerovlaGoToNode.land_finished(node, future)
        self.assertEqual(node.state, 'STOPPED')
        self.assertFalse(node.land_success)
        self.assertTrue(node.stop_goal_ack)
        self.assertIsNone(node.goal_handle)
        node.publish_status.assert_not_called()

    def test_late_acceptance_is_canceled(self):
        node = Mock(phase='STOPPING')
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = Future()
        future = Future()
        future.set_result(handle)
        AerovlaGoToNode.land_accepted(node, future)
        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(node.send_pending)

    def test_landing_and_missing_server_have_bounded_timeout(self):
        for phase in ('LAND_READY', 'LANDING'):
            node = Mock(phase=phase, deadline=0.0)
            AerovlaGoToNode.execution_tick(node)
            node.fail.assert_called_once_with('LAND_TIMEOUT')
