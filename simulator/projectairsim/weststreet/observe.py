#!/usr/bin/env python3
"""Load one uncommanded native drone; forward telemetry and select HUD Global follow."""
import argparse,copy,json,math,sys,time,threading
from pathlib import Path
from urllib.request import Request,ProxyHandler,build_opener
sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from simulator.projectairsim.display_session import views_from_scene
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient

def main():
 import commentjson
 from projectairsim import ProjectAirSimClient,World
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--native-source',type=Path,required=True)
 p.add_argument('--output',type=Path,required=True)
 p.add_argument('--topic-port',type=int,default=19889)
 p.add_argument('--service-port',type=int,default=19890)
 p.add_argument('--display-port',type=int,default=19891)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 base=a.native_source/'client/python/example_user_scripts/sim_config'
 robot=commentjson.loads((base/'robot_quadrotor_fastphysics.jsonc').read_text())
 down=next(s for s in robot['sensors'] if s.get('id')=='DownCamera')
 front=copy.deepcopy(down);front['id']='FrontCamera';front['origin']={'xyz':'0.15 0 -0.02','rpy-deg':'0 0 0'}
 robot['sensors'].append(front)
 for sensor in [down,front]:
  sensor['capture-interval']=.1
  for cap in sensor['capture-settings']:
   enabled=cap['image-type'] in ([0,2] if sensor is front else [0])
   cap.update({'width':320,'height':180,'capture-enabled':enabled,'streaming-enabled':enabled})
 scene=commentjson.loads((base/'scene_basic_drone.jsonc').read_text())
 scene['id']='SceneWestStreet'
 scene['actors']=[{'type':'robot','name':'Drone1','origin':{'xyz':'8 -45 -0.3','rpy-deg':'0 0 90'},'robot-config':'robot_weststreet.jsonc'}]
 # Physics runs naturally, with no enable-api-control/arm/takeoff/motion calls.
 (a.output/'robot_weststreet.jsonc').write_text(json.dumps(robot,indent=2))
 (a.output/'scene_weststreet.jsonc').write_text(json.dumps(scene,indent=2))
 client=ProjectAirSimClient(port_topics=a.topic_port,port_services=a.service_port)
 client.connect();client.socket_services.recv_timeout=5000
 rpc=ProjectAirSimExtensionClient(port=a.display_port)
 opener=build_opener(ProxyHandler({})).open
 images={};lock=threading.Lock();samples=[]
 try:
  world=World(client,'scene_weststreet.jsonc',sim_config_path=str(a.output.resolve()),delay_after_load_sec=2)
  views,channels=views_from_scene(world.get_configuration())
  rpc.call('RegisterViews',views=views)
  rpc.call('SetTitle',text='街区观察',side='left')
  rpc.call('SetTitle',text='街区场景',side='right')
  rpc.call('SetView',vehicle_id='__global__')
  def receive(source,data):
   with lock:
    entry=images.setdefault(source,{'frames':0})
    entry['frames']+=1
    entry['keys']=list(data) if isinstance(data,dict) else []
    if isinstance(data,dict):
     for key in ['width','height','time_stamp','timestamp']:
      if key in data:entry[key]=data[key]
     # Keep one real frame per source for offline inspection, not synthetic data.
     if entry['frames']==1:
      try:
       import pickle
       with (a.output/('_'.join(source.split('/')[-2:])+'.pickle')).open('wb') as f:pickle.dump(data,f)
      except Exception as exc:entry['save_error']=str(exc)
  for channel in channels:
   client.subscribe(channel.source,lambda _,data,source=channel.source:receive(source,data))
  started=time.monotonic();last_write=0
  print('WEST_OBSERVATION_READY: one drone, no control commands, Global follows runner',flush=True)
  while True:
   state=client.request({'method':'/Sim/SceneWestStreet/robots/Drone1/GetGroundTruthKinematics','params':{},'version':1.0})
   velocity=state['twist']['linear'];speed=math.sqrt(sum(velocity[k]**2 for k in ['x','y','z']))
   with opener(Request(f'http://127.0.0.1:{a.display_port}/aerohub/telemetry',data=json.dumps({'vehicle':'Drone1','speed_mps':speed}).encode(),headers={'Content-Type':'application/json'}),timeout=3) as response:response.read()
   if time.monotonic()-last_write>1:
    samples.append({'elapsed_seconds':time.monotonic()-started,'kinematics':state,'speed_mps':speed})
    with lock:received=copy.deepcopy(images)
    api_control=client.request({'method':'/Sim/SceneWestStreet/robots/Drone1/IsApiControlEnabled','params':{},'version':1.0})
    landed=client.request({'method':'/Sim/SceneWestStreet/robots/Drone1/GetLandedState','params':{},'version':1.0})
    (a.output/'observation.json').write_text(json.dumps({'control_commands_sent':[],'api_control_enabled':api_control,'landed_state':landed,'views':views,'images':received,'hud':rpc.call('GetDisplayState'),'samples':samples[-120:]},indent=2))
    last_write=time.monotonic()
   time.sleep(.1)
 finally:client.disconnect()
if __name__=='__main__':main()
