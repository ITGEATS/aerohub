"""Real ROS executor plus controlled polynomial/future tests; no Behavior changes."""
from concurrent.futures import Future
import importlib.util
from pathlib import Path
import sys,time
from types import SimpleNamespace
from unittest.mock import Mock
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
import yaml
from uav_elastic_tracker_adapter.core import enu_to_ned,evaluate,suffix,stamp_ns
from uav_elastic_tracker_adapter.node import Adapter
from uav_elastic_tracker_adapter.native_execution import NativeAdapter

def baseline(flag,name,default):
 if flag not in sys.argv:return default
 i=sys.argv.index(flag);path=sys.argv[i+1];del sys.argv[i:i+2]
 spec=importlib.util.spec_from_file_location('uav_elastic_tracker_adapter.'+name,path)
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 return getattr(module,default.__name__)

Adapter=baseline('--adapter-source','rolling_old_node',Adapter)
NativeAdapter=baseline('--native-source','rolling_old_native',NativeAdapter)
ROOT=Path(__file__).resolve().parents[6]
CONFIG=ROOT/'src/tool/navigation/planner/elastic_tracker/ros2_adapter/bridge/config/native_source.yaml'
PARAMETERS=yaml.safe_load(CONFIG.read_text())['planner']['parameters']
SECOND=1_000_000_000

def curve(identifier,start,cubic=0.):
 # Same unit polynomial basis as test_core.py; P/V/A are zero at local time 0.
 return dict(traj_id=identifier,start_time=list(divmod(start,SECOND)),generated_at=[100,0],
  hover=False,order=5,duration=[3.],yaw=0.,coef_x=[0.,0.,cubic,0.,0.,0.],coef_y=[0.]*6,coef_z=[0.]*6)

def handoff(start_offset=30_000_000):
 now=100*SECOND
 # Source timing is planning_nodelet.cpp:268,512 (unchanged 0.03s).
 source=curve(2,now+start_offset,1.)
 previous=curve(1,99*SECOND)
 client=SimpleNamespace(ready=True,pending=set(),closed=False,failed=False,revision=1,
  acknowledged_revision=1,submit=Mock())
 node=SimpleNamespace(queued=source,client=client,reference_ready=True,parameters=PARAMETERS,
  last_curve=(suffix(previous,99*SECOND,PARAMETERS['sample_dt'],0.),99*SECOND),
  last_source=previous,
  first_start=99*SECOND,yaw=0.,task='rolling-test',vehicle='drone0',late_updates=0,submitted=0,
  fail=Mock(),_fresh_inputs=Mock(return_value=True),get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now)))
 return node,source,now

def submitted_valid(node):
 valid=node.client.submit.call_args.args[0].valid_from
 return valid.sec*SECOND+valid.nanosec

class RollingHandoffTest(unittest.TestCase):
 def test_update_rejection_discards_candidate_and_keeps_coverage_of_acknowledged_curve(self):
  node,source,now=handoff(200_000_000)
  previous=node.last_curve
  node.submission=(source,suffix(source,stamp_ns(source['start_time']),PARAMETERS['sample_dt'],0.),
                   stamp_ns(source['start_time']))
  node.rejected_updates=0
  node.discard_offer=Mock()
  node.check_coverage=Mock()
  self.assertTrue(NativeAdapter.execution_update_rejected(node))
  node.discard_offer.assert_called_once_with(source['traj_id'],'execution_update_rejected')
  node._fresh_inputs.assert_called_once_with(now,'after_update_rejected')
  node.check_coverage.assert_called_once()
  self.assertIsNone(node.submission)
  self.assertIsNone(node.queued)
  self.assertIs(node.last_curve,previous)
  self.assertEqual(node.rejected_updates,1)

 def test_acknowledged_handoff_keeps_source_epoch_and_commits_after_reply(self):
  node,source,_=handoff(200_000_000)
  node.execution_handoff=True
  node.submission=None
  node.accepted=Mock()
  node.native_topic='/elastic/trajectory'
  node.submit_queued=Mock()
  previous=node.last_source
  NativeAdapter.submit_queued(node)
  self.assertEqual(submitted_valid(node),stamp_ns(source['start_time']))
  self.assertIs(node.last_source,previous)
  node.client.pending.add('drone0')
  NativeAdapter.execution_ready(node)
  node.accepted.publish.assert_not_called()
  self.assertIs(node.last_source,previous)
  node.client.pending.clear()
  NativeAdapter.execution_ready(node)
  self.assertIs(node.last_source,source)
  self.assertEqual(node.last_curve[1],stamp_ns(source['start_time']))
  self.assertEqual(node.accepted.publish.call_args.args[0].data,source['traj_id'])
  self.assertIsNone(node.submission)

 def test_acknowledged_handoff_discards_late_candidate_without_changing_predecessor(self):
  node,source,_=handoff(30_000_000)
  node.execution_handoff=True
  node.discard_offer=Mock()
  previous=node.last_source
  NativeAdapter.submit_queued(node)
  node.client.submit.assert_not_called()
  node.discard_offer.assert_called_once_with(source['traj_id'],'insufficient_lead')
  self.assertIs(node.last_source,previous)
  self.assertIsNone(node.queued)

 def test_rolling_replacement_is_admitted_after_the_lead(self):
  node,source,now=handoff();NativeAdapter.submit_queued(node)
  node.client.submit.assert_called_once()
  actual=submitted_valid(node)
  self.assertEqual(actual,now+round(PARAMETERS['valid_from_lead']*1e9))
  # The first point must be the source curve at the admitted offset, never a
  # relabelled point of its own earlier splice epoch.
  local=(actual-stamp_ns(source['start_time']))/SECOND
  self.assertAlmostEqual(local,PARAMETERS['valid_from_lead']-0.03,places=12)
  first=node.client.submit.call_args.args[0].vehicle_trajectories[0].points[0]
  for derivative,key in enumerate(('position','velocity','acceleration')):
   expected=enu_to_ned([evaluate(source['coef_'+axis],5,0,local,derivative) for axis in 'xyz'])
   for axis,value in zip('xyz',expected):
    self.assertAlmostEqual(getattr(getattr(first,key),axis),value,places=12)

 def test_late_native_splice_is_led_forward_not_rejected(self):
  node,source,now=handoff(-20_000_000);NativeAdapter.submit_queued(node)
  node.client.submit.assert_called_once()
  self.assertEqual(submitted_valid(node),now+round(PARAMETERS['valid_from_lead']*1e9))
  self.assertEqual(node.late_updates,0);node.fail.assert_not_called()

 def test_source_looking_further_ahead_keeps_its_own_epoch(self):
  node,source,now=handoff(2*SECOND);NativeAdapter.submit_queued(node)
  self.assertEqual(submitted_valid(node),stamp_ns(source['start_time']))
  # Still the exact source epoch: P/V/A are zero there, as in the source.
  first=node.client.submit.call_args.args[0].vehicle_trajectories[0].points[0]
  self.assertEqual([getattr(getattr(first,k),a) for k in ('position','velocity','acceleration')
                    for a in ('x','y','z')],[0.]*9)

 def test_first_admission_uses_the_same_lead(self):
  node,source,now=handoff();node.client.revision=0;node.last_curve=None;node.first_start=None
  node.odom_stamp=now-10_000_000;node.position=[0.,0.,0.]
  NativeAdapter.submit_queued(node)
  self.assertEqual(submitted_valid(node),now+round(PARAMETERS['valid_from_lead']*1e9))
  self.assertEqual(node.initial_handoff['now_ns'],now)

 def test_pending_execution_ack_does_not_consume_new_source(self):
  node,source,now=handoff();node.client.pending={'drone0'}
  NativeAdapter.submit_queued(node)
  node.client.submit.assert_not_called();self.assertIs(node.queued,source)

 def test_same_clock_sample_defers_rolling_update_until_time_advances(self):
  node,source,now=handoff()
  NativeAdapter.submit_queued(node)
  accepted_start=submitted_valid(node)
  node.client.submit.reset_mock()
  node.queued=curve(source['traj_id']+1,stamp_ns(source['start_time']),2.)
  latest=node.queued
  NativeAdapter.submit_queued(node)
  node.client.submit.assert_not_called()
  self.assertIs(node.queued,latest)
  node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now+9_000_000))
  NativeAdapter.submit_queued(node)
  node.client.submit.assert_called_once()
  self.assertEqual(submitted_valid(node),accepted_start+9_000_000)
  self.assertIs(node.last_source,latest)
  self.assertIsNone(node.queued)

 def test_completed_rpc_wakes_executor_without_next_timer_tick(self):
  class Probe(Adapter):
   def _publish_result(self,result,now):self.received.append(result)
  rclpy.init(args=['--ros-args','-p','resolved_config:='+str(CONFIG)])
  node=Probe();executor=SingleThreadedExecutor();executor.add_node(node)
  try:
   for timer in node.timers:timer.cancel()
   node.rpc_worker.shutdown(wait=True)
   node.rpc_worker=Mock();pending=Future();node.rpc_worker.submit.return_value=pending
   node.target_stamp=node.odom_stamp=node.get_clock().now().nanoseconds;node.yaw=0.;node.received=[]
   node.tick();response={'ok':True,'trajectory':None};pending.set_result(response)
   # Existing fixture observation budget from native_ros_smoke.py:115.
   end=time.monotonic()+6.
   while not node.received and time.monotonic()<end:executor.spin_once(timeout_sec=.02)
   self.assertEqual(node.received,[response])
   self.assertEqual(node.rpc_worker.submit.call_count,1,'completion must not create an RPC busy loop')
  finally:
   executor.remove_node(node);node.destroy_node();executor.shutdown();rclpy.shutdown()

if __name__=='__main__':unittest.main(verbosity=2)
