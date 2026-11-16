#!/usr/bin/env python3
"""Integration checks against actual scene RPC; never controls a UAV."""
import argparse,json,math,time
from pathlib import Path
from actor_control import SceneControlClient
from simulator.projectairsim.extension_client import ExtensionRPCError

def distance(a,b):return math.dist([a['location_ue_cm'][k] for k in ('x','y','z')],[b['location_ue_cm'][k] for k in ('x','y','z')])
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',type=int,default=19892);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 rpc=SceneControlClient(port=a.port);steps=[];result={'passed':False,'steps':steps}
 def save(name,state):steps.append({'step':name,'wall_time':time.time(),'state':state});return state
 try:
  initial=save('initial',rpc.state());assert initial['state']=='paused' and abs(initial['time_seconds'])<.01,'Runner must start paused at t=0'
  time.sleep(1);still=save('initial_wait',rpc.state());assert distance(initial,still)<.1 and abs(still['time_seconds']-initial['time_seconds'])<.001
  save('start',rpc.control('start'));time.sleep(1.2);moving=save('moving',rpc.state())
  assert moving['state']=='running' and distance(initial,moving)>100,'Runner did not actually move'
  repeated=save('repeated_start',rpc.control('start'));assert repeated['time_seconds']>=moving['time_seconds']-.01,'Repeated start rewound sequence'
  stopped=save('stop',rpc.control('stop'));time.sleep(1);frozen=save('frozen',rpc.state())
  assert frozen['state']=='paused' and distance(stopped,frozen)<.1 and abs(frozen['time_seconds']-stopped['time_seconds'])<.001
  again=save('repeated_stop',rpc.control('stop'));assert distance(frozen,again)<.1
  save('resume',rpc.control('start'));time.sleep(.7);resumed=save('resumed',rpc.state());assert resumed['time_seconds']>frozen['time_seconds']+.1
  reset=save('reset',rpc.control('reset'));assert reset['state']=='paused' and abs(reset['time_seconds'])<.001 and distance(initial,reset)<.1
  for method,params in [('ControlActor',{'actor_id':'Drone1','event':'start'}),('ControlActor',{'actor_id':'runner','event':'warp'}),('UnknownMethod',{'actor_id':'runner'})]:
   try:rpc.call(method,**params)
   except ExtensionRPCError as exc:steps.append({'step':'rejected_invalid_request','method':method,'params':params,'code':exc.code})
   else:raise AssertionError('Invalid actor/event/method accepted')
  final=save('after_invalid',rpc.state());assert final['state']=='paused' and distance(initial,final)<.1
  result['passed']=True
 except Exception as exc:
  result['error']=repr(exc);raise
 finally:
  try:save('cleanup_stop',rpc.control('stop'))
  finally:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2))
 print('PASS: paused startup, actual move/freeze/resume/reset, idempotence and rejected invalid requests')
if __name__=='__main__':main()
