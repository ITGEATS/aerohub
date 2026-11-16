#!/usr/bin/env python3
"""Explicit opt-in SimpleFlight smoke: takeoff, actor start, 5m forward, hover, land.
Uses the existing scene via service-only SDK connection; never reloads it.
"""
import argparse,asyncio,json,math,sys,time,traceback
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient
from actor_control import SceneControlClient

async def run(a):
 from projectairsim import ProjectAirSimClient
 if not a.execute_flight:raise RuntimeError('Requires --execute-flight; this sends real simulation control commands')
 if not 0<a.forward_meters<=5 or not 0<a.speed<=1:raise ValueError('Maximum 5m forward at 1m/s')
 a.output.mkdir(parents=True,exist_ok=False)
 policy={'backend':'ProjectAirSim SimpleFlight; not PX4/AS2','takeoff_height_m':3,'forward_meters':a.forward_meters,'speed_mps':a.speed,'hover_seconds':2,'max_height_above_start_m':4.5,'max_x_offset_m':2,'max_y_offset_m':9,'position_tolerance_m':.75,'hover_speed_limit_mps':.25,'motion_mode':a.motion_mode,'touchdown_max_height_error_m':.15,'landing_confirmation':'Land returned true + physical ground height/speed, then disarm and require final landed flag=0'}
 (a.output/'test-policy.json').write_text(json.dumps(policy,indent=2))
 result={'policy':policy,'passed':False,'events':[],'samples':[]};phase='preflight';initial=None;armed=False;control=False;run_started=False
 client=ProjectAirSimClient(port_services=a.service_port);client.connect_services();client.socket_services.recv_timeout=45000
 hud=ProjectAirSimExtensionClient(port=a.display_port);actor=SceneControlClient(port=a.scene_port)
 prefix='/Sim/SceneWestStreet/robots/Drone1/'
 def flush():(a.output/'flight.json').write_text(json.dumps(result,indent=2))
 def event(name,**details):
  result['events'].append({'name':name,'wall_time':time.time(),**details});flush();print(name,details,flush=True)
 def request(name,params=None):return client.request({'method':prefix+name,'params':params or {},'version':1.0})
 def sample(guard=True):
  k=request('GetGroundTruthKinematics');pos=k['pose']['position'];vel=k['twist']['linear'];speed=math.sqrt(sum(vel[c]**2 for c in 'xyz'))
  sample={'wall_time':time.time(),'phase':phase,'position':pos,'velocity':vel,'speed_mps':speed,'native_timestamp':k['time_stamp']}
  result['samples'].append(sample)
  if guard and initial:
   p0=initial['pose']['position']
   if abs(pos['x']-p0['x'])>2 or abs(pos['y']-p0['y'])>9 or not -.5<=p0['z']-pos['z']<=4.5:
    raise RuntimeError('Flight exceeded predeclared spatial envelope')
  return k,sample
 async def command(name,params,limit=35,guard=True):
  event('command',method=name,params=params)
  task=await client.request_async({'method':prefix+name,'params':params,'version':1.0})
  deadline=time.monotonic()+limit
  try:
   while not task.done():
    sample(guard)
    if time.monotonic()>deadline:raise TimeoutError(name+' deadline')
    await asyncio.sleep(.1)
   value=await task
   event('command_complete',method=name,result=value)
   if value is False:raise RuntimeError(name+' reported false')
   return value
  finally:
   if not task.done():task.cancel()
 async def settle(seconds=2,guard=True):
  end=time.monotonic()+seconds;states=[]
  while time.monotonic()<end:
   _,s=sample(guard);states.append(s);await asyncio.sleep(.1)
  return states
 try:
  initial,first=sample();result['initial_kinematics']=initial
  assert request('GetLandedState')==0,'Drone must initially be landed'
  assert request('IsApiControlEnabled') is False,'Another controller already owns drone'
  assert first['speed_mps']<.1,'Drone must initially be stationary'
  paused=actor.control('reset');assert paused['state']=='paused' and abs(paused['time_seconds'])<.001
  hud.call('SetView',vehicle_id='Drone1',camera_id='Chase',image_type='RGB')
  event('runner_waiting_for_takeoff',actor=paused)
  assert request('EnableApiControl'),'Cannot enable API control';control=True
  event('command',method='Arm');assert request('Arm'),'Cannot arm';armed=True
  hud.call('ControlRun',vehicle_id='Drone1',event='start');run_started=True
  phase='takeoff'
  takeoff_z=initial['pose']['position']['z']-3
  if a.motion_mode=='velocity':
   deadline=time.monotonic()+25
   while True:
    k,_=sample();remaining=k['pose']['position']['z']-takeoff_z
    if remaining<.12:break
    if time.monotonic()>deadline:raise TimeoutError('Velocity takeoff altitude deadline')
    await command('MoveByVelocity',{'vx':0.0,'vy':0.0,'vz':-min(.5,max(.15,remaining)), 'duration':.4,'drivetrain':0,'yaw_is_rate':True,'yaw':0.0},10)
   await command('MoveByVelocityZ',{'vx':0.0,'vy':0.0,'z':takeoff_z,'duration':1.0,'drivetrain':0,'yaw_is_rate':True,'yaw':0.0},10)
  else:await command('Takeoff',{'timeout_sec':25},35)
  states=await settle(1)
  current,latest=sample();p0=initial['pose']['position'];p=current['pose']['position'];height=p0['z']-p['z']
  assert 2.5<=height<=3.8,'Actual takeoff altitude not reached'
  assert request('GetLandedState')==1,'Drone not airborne'
  event('takeoff_confirmed',height_m=height,position=p)
  started=actor.control('start');assert started['state']=='running'
  event('runner_started_after_takeoff',actor=started)
  q=initial['pose']['orientation'];yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
  target={'x':p['x']+a.forward_meters*math.cos(yaw),'y':p['y']+a.forward_meters*math.sin(yaw),'z':p['z']}
  phase='forward'
  if a.motion_mode=='velocity':
   deadline=time.monotonic()+30
   while True:
    k,_=sample();actual=k['pose']['position'];dx=target['x']-actual['x'];dy=target['y']-actual['y'];dist=math.hypot(dx,dy)
    if dist<.15:break
    if time.monotonic()>deadline:raise TimeoutError('Forward position deadline')
    scale=min(1.0,a.speed/max(dist,.001))
    await command('MoveByVelocityZ',{'vx':dx*scale,'vy':dy*scale,'z':target['z'],'duration':.4,'drivetrain':0,'yaw_is_rate':True,'yaw':0.0},10)
  else:await command('MoveToPosition',dict(target,velocity=a.speed,timeout_sec=30,drivetrain=0,yaw_is_rate=True,yaw=0.0,lookahead=-1.0,adaptive_lookahead=1.0),40)
  current,latest=sample();p=current['pose']['position'];error=math.dist([p[c] for c in 'xyz'],[target[c] for c in 'xyz'])
  assert error<.75,'Forward target not reached'
  event('forward_confirmed',target=target,position=p,error_m=error)
  phase='hover'
  if a.motion_mode=='velocity':await command('MoveByVelocityZ',{'vx':0.0,'vy':0.0,'z':target['z'],'duration':2.0,'drivetrain':0,'yaw_is_rate':True,'yaw':0.0},10)
  else:await command('Hover',{},15)
  states=await settle(2)
  assert max(s['speed_mps'] for s in states[-10:])<.25,'Drone did not stop'
  drift=math.dist([states[0]['position'][c] for c in 'xyz'],[states[-1]['position'][c] for c in 'xyz'])
  assert drift<.5,'Hover drift exceeded limit'
  event('hover_confirmed',drift_m=drift,max_final_speed_mps=max(s['speed_mps'] for s in states[-10:]))
  phase='land';await command('Land',{'timeout_sec':30},40);await settle(1)
  ground,ground_sample=sample()
  assert abs(ground['pose']['position']['z']-p0['z'])<.15 and ground_sample['speed_mps']<.1,'Physical touchdown not confirmed'
  event('landing_confirmed',landed_flag_before_disarm=request('GetLandedState'),actual_ground_height_error_m=abs(ground['pose']['position']['z']-p0['z']))
  assert request('Disarm'),'Disarm rejected';armed=False
  assert request('DisableApiControl'),'Cannot release API control';control=False
  await settle(1)
  assert request('GetLandedState')==0,'Final landed flag not confirmed'
  hud.call('ControlRun',vehicle_id='Drone1',event='finish');run_started=False
  stopped=actor.control('stop');assert stopped['state']=='paused'
  event('runner_stopped',actor=stopped)
  final,latest=sample();assert latest['speed_mps']<.1,'Drone not stationary after landing'
  result.update({'passed':True,'final_kinematics':final,'final_api_control_enabled':request('IsApiControlEnabled'),'final_landed_state':request('GetLandedState'),'final_actor_state':stopped})
 except BaseException as exc:
  result['error']=repr(exc);result['traceback']=traceback.format_exc();flush()
  phase='recovery'
  if control or armed:
   try:
    if not client.state:client.connect_services()
    if request('GetLandedState')!=0:
     try:await command('Hover',{},10,guard=False)
     except Exception as e:event('recovery_hover_error',error=repr(e))
     await command('Land',{'timeout_sec':30},40,guard=False)
    ground=request('GetGroundTruthKinematics')
    ground_confirmed=initial and abs(ground['pose']['position']['z']-initial['pose']['position']['z'])<.15 and sum(ground['twist']['linear'][axis]**2 for axis in 'xyz')<.01
    if request('GetLandedState')==0 or ground_confirmed:
     event('recovery_landed',physical_ground_confirmed=bool(ground_confirmed));request('Disarm');armed=False;request('DisableApiControl');control=False
    else:event('recovery_incomplete',warning='Airborne: refusing to disarm')
   except BaseException as e:event('recovery_error',error=repr(e),warning='State unverified; do not stop simulator until inspected')
  raise
 finally:
  try:result['final_actor_state']=actor.control('stop')
  except Exception as e:result['actor_cleanup_error']=repr(e)
  if run_started:
   try:hud.call('ControlRun',vehicle_id='Drone1',event='finish')
   except Exception:pass
  flush();client.disconnect()
 print('PASS: takeoff -> actor start -> forward -> hover -> land; disarmed and API released',flush=True)

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--execute-flight',action='store_true');p.add_argument('--output',type=Path,required=True)
 p.add_argument('--service-port',type=int,default=19890);p.add_argument('--display-port',type=int,default=19891);p.add_argument('--scene-port',type=int,default=19892)
 p.add_argument('--motion-mode',choices=['native_path','velocity'],default='native_path')
 p.add_argument('--forward-meters',type=float,default=5);p.add_argument('--speed',type=float,default=1)
 asyncio.run(run(p.parse_args()))
