"""Non-actuating regression of production Adapter.tick with controlled futures.

Real ROS imports are required (no skip); no Node, ACK or flight Goal is created.
--adapter-source permits the same assertions against the saved pre-fix source.
"""
import importlib.util
from concurrent.futures import Future
from pathlib import Path
import sys
from types import MethodType,SimpleNamespace
import unittest
from unittest.mock import Mock
import yaml

from uav_elastic_tracker_adapter.contracts import InputDiagnostics
from uav_elastic_tracker_adapter.node import Adapter

if '--adapter-source' in sys.argv:
 i=sys.argv.index('--adapter-source');path=sys.argv[i+1];del sys.argv[i:i+2]
 spec=importlib.util.spec_from_file_location('uav_elastic_tracker_adapter.rpc_baseline',path)
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);Adapter=module.Adapter

ROOT=Path(__file__).resolve().parents[6]
PARAMETERS=yaml.safe_load((ROOT/'src/tool/navigation/planner/elastic_tracker/ros2_adapter/bridge/config/native_source.yaml').read_text())['planner']['parameters']
SECOND=1_000_000_000

def probe():
 node=SimpleNamespace(failed=False,target_stamp=100*SECOND,odom_stamp=100*SECOND,yaw=0.,
  now_ns=100*SECOND,parameters=PARAMETERS,task='rpc-freshness',last_id=None,
  pending_rpc=None,request_snapshot=None,rpc_worker=Mock(),rpc_ready=Mock(),telemetry=Mock(),failures=[],
  _publish_result=Mock(),get_parameter=lambda name:SimpleNamespace(value='127.0.0.1' if name=='proxy_host' else 28500))
 logger=Mock();node.diagnostics=InputDiagnostics(logger)
 node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=node.now_ns))
 def fail(reason,detail):node.failed=True;node.failures.append((reason,detail))
 node.fail=fail
 node.tick=MethodType(Adapter.tick,node);node._fresh_inputs=MethodType(Adapter._fresh_inputs,node)
 if hasattr(Adapter,'on_rpc_ready'):node.on_rpc_ready=MethodType(Adapter.on_rpc_ready,node)
 node.rpc_worker.submit.side_effect=lambda *args:Future()
 return node

def delayed(node):
 node.tick();pending=node.pending_rpc
 # Cross the unchanged input-age boundary by one nanosecond; this is a test input.
 node.now_ns+=round(PARAMETERS['target_timeout']*SECOND)+1
 node.target_stamp=node.odom_stamp=node.now_ns
 return pending

class RpcFreshnessTest(unittest.TestCase):
 def test_delayed_success_uses_current_inputs_and_preserves_response(self):
  node=probe();pending=delayed(node);response={'ok':True,'trajectory':None}
  pending.set_result(response);node.tick()
  self.assertFalse(node.failed,node.failures)
  node._publish_result.assert_called_once_with(response,node.now_ns)
  self.assertIs(node._publish_result.call_args.args[0],response)

 def test_transport_error_is_not_target_timeout_and_next_tick_can_poll(self):
  node=probe();pending=delayed(node);pending.set_exception(TimeoutError('proxy read timed out'))
  node.tick()
  self.assertFalse(node.failed,node.failures)
  self.assertEqual(node.diagnostics.counts['rpc_error:TimeoutError'],1)
  node._publish_result.assert_not_called()
  node.tick()
  self.assertEqual(node.rpc_worker.submit.call_count,2)

 def test_real_target_loss_stops_while_rpc_pending(self):
  node=probe();node.tick()
  node.now_ns+=round(PARAMETERS['target_timeout']*SECOND)+1;node.odom_stamp=node.now_ns
  node.tick()
  self.assertTrue(node.failed)
  self.assertEqual(node.failures[0][0],'TARGET_TIMEOUT')
  node._publish_result.assert_not_called()

 def test_real_target_loss_stops_on_successful_rpc_return(self):
  node=probe();node.tick();node.pending_rpc.set_result({'ok':True,'trajectory':None})
  node.now_ns+=round(PARAMETERS['target_timeout']*SECOND)+1;node.odom_stamp=node.now_ns
  node.tick()
  self.assertTrue(node.failed)
  self.assertEqual(node.failures[0][0],'TARGET_TIMEOUT')
  node._publish_result.assert_not_called()

 def test_real_odometry_loss_remains_state_timeout(self):
  node=probe();node.tick()
  node.now_ns+=round(PARAMETERS['input_max_age']*SECOND)+1;node.target_stamp=node.now_ns
  node.tick()
  self.assertTrue(node.failed)
  self.assertEqual(node.failures[0][0],'STATE_TIMEOUT')

 def test_pending_rpc_has_single_owner_while_inputs_advance(self):
  node=probe();pending=delayed(node);node.tick();node.tick()
  self.assertFalse(node.failed,node.failures)
  self.assertIs(node.pending_rpc,pending)
  self.assertEqual(node.rpc_worker.submit.call_count,1)

if __name__=='__main__':unittest.main(verbosity=2)
