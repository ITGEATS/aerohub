"""Direct DDS admission/ownership and lossless native message regressions."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import unittest
from quadrotor_interfaces.msg import PolyTraj
from uav_elastic_tracker_adapter.native_execution import NativeAdapter
from uav_elastic_tracker_adapter.native_message import trajectory_record


def fixture():
    return NS(enabled=True, failed=False, reference_ready=True,
              client=NS(closed=False,failed=False,pending=set(),revision=1),
              get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000)),
              _fresh_inputs=Mock(return_value=True), _publish_result=Mock(),
              execution_ready=Mock())


class DirectNativeTest(unittest.TestCase):
    def test_dds_keeps_source_epoch_and_coefficients(self):
        node=fixture();message=PolyTraj()
        message.traj_id=17;message.order=5;message.start_time.sec=100
        message.start_time.nanosec=30_000_000;message.duration=[3.]
        message.coef_x=message.coef_y=message.coef_z=[0.,0.,0.,0.,1.,.5]
        NativeAdapter.on_native_trajectory(node,message)
        result,now=node._publish_result.call_args.args
        self.assertEqual(now,100_000_000_000)
        self.assertEqual(result['trajectory']['start_time'],[100,30_000_000])
        self.assertEqual(result['trajectory']['coef_x'],list(message.coef_x))
        self.assertEqual(result['trajectory'],trajectory_record(message,now))
        node.execution_ready.assert_called_once()

    def test_unacknowledged_update_waits_for_next_publication(self):
        node=fixture();node.client.pending={'drone0'}
        NativeAdapter.on_native_trajectory(node,PolyTraj())
        node._publish_result.assert_not_called()
        node.client.pending.clear()
        NativeAdapter.on_native_trajectory(node,PolyTraj(traj_id=2))
        self.assertEqual(node._publish_result.call_args.args[0]['trajectory']['traj_id'],2)

    def test_first_live_reference_is_required_before_rolling(self):
        node=fixture();node.reference_ready=False
        NativeAdapter.on_native_trajectory(node,PolyTraj())
        node._publish_result.assert_not_called()

    def test_hover_preempts_pending_update(self):
        node=fixture();node.client.pending={'drone0'};node.reference_ready=False
        NativeAdapter.on_native_trajectory(node,PolyTraj(hover=True,hover_p=[0.,0.,0.]))
        node._publish_result.assert_called_once()

    def test_closed_or_disabled_owner_consumes_nothing(self):
        for disabled,closed in [(True,False),(False,True)]:
            node=fixture();node.enabled=not disabled;node.client.closed=closed
            NativeAdapter.on_native_trajectory(node,PolyTraj())
            node._publish_result.assert_not_called()

    def test_real_input_health_is_still_required(self):
        node=fixture();node._fresh_inputs.return_value=False
        NativeAdapter.on_native_trajectory(node,PolyTraj())
        node._publish_result.assert_not_called()


if __name__=='__main__':
    unittest.main()
