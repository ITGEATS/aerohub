#!/usr/bin/env python3
"""PC four-role street validation using existing manager, flight_board and AS2."""
import argparse,copy,datetime,json,os,re,signal,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient

DOMAIN=197;INSTANCE=30;SYSTEM=31

def call(args,**kw):return subprocess.run(list(map(str,args)),check=True,**kw)
def capture(args):return call(args,capture_output=True,text=True).stdout

def prepare(a,out):
 profile=json.loads(a.base_profile.read_text());scene_path=Path(profile['scene_config'])
 scene=json.loads(scene_path.read_text());robot=json.loads((scene_path.parent/scene['actors'][0]['robot-config']).read_text())
 if robot['controller']['type']!='px4-api':raise RuntimeError('Base must be the existing verified PX4 scene')
 scene['id']='SceneWestStreetPx4';scene['actors']=[{'type':'robot','name':'Drone1','origin':{'xyz':'8 -45 -0.3','rpy-deg':'0 0 90'},'robot-config':'robot_px4.jsonc'}]
 (out/'robot_px4.jsonc').write_text(json.dumps(robot,indent=2));(out/'scene_px4.jsonc').write_text(json.dumps(scene,indent=2))
 profile.update(binary=str(a.ue_root/'Engine/Binaries/Linux/UnrealEditor'),scene_config=str(out/'scene_px4.jsonc'),editor_project=str(a.project),ports={'topics':19889,'services':19890})
 profile['args']=[str(a.project),'/Game/Aerohub/WestStreetCircuit','-game','-windowed','-ResX=1280','-ResY=720','-nosound','-noraytracing']
 profile['vehicles']=[{'name':'Drone1','namespace':'drone0','px4':{'tcp_port':4572,'control_local_port':14552,'control_remote_port':14592,'lockstep':True}}]
 profile['sensor_options']={'Drone1':{'DownCamera':copy.deepcopy(profile['sensor_options']['Drone1']['DownCamera'])}}
 profile['display'].update(port=19891,global_camera_tag='WestRunnerGlobalFollow')
 profile['adapter']['environment'].update(ROS_DOMAIN_ID=str(DOMAIN),PROJECTAIRSIM_DDS_PROFILE=str(ROOT/'deployment/airsim/config/dds/fastdds_udp.xml'))
 (out/'profile.json').write_text(json.dumps(profile,indent=2));(out/'scene-control.json').write_text('{"port":19892}')
 (out/'fleet.yaml').write_text('vehicles:\n  drone0:\n    fmu_prefix: /drone0\n    target_system_id: 31\n')
 libraries=[a.ue_root/'Engine/Binaries/Linux',a.project.parent/'Plugins/ProjectAirSim/Binaries/Linux']
 libraries += [a.ue_root/'Engine/Plugins'/p/'Binaries/Linux' for p in ['Experimental/ChaosVehiclesPlugin','Media/PixelStreaming','Runtime/ProceduralMeshComponent','Runtime/SunPosition']]
 env=os.environ.copy();env['LD_LIBRARY_PATH']=':'.join(map(str,libraries));env['AEROHUB_SCENE_CONTROL_CONFIG']=str(out/'scene-control.json')
 if 'DISPLAY' not in env:env['DISPLAY']=':1'
 if 'XAUTHORITY' not in env:
  path=Path(f'/run/user/{os.getuid()}/gdm/Xauthority')
  if path.is_file():env['XAUTHORITY']=str(path)
 return env

def main():
 p=argparse.ArgumentParser(description=__doc__)
 for name in ['base-profile','ue-root','project','workstation-overlay','output']:p.add_argument('--'+name,type=Path,required=True)
 p.add_argument('--ground-only',action='store_true');a=p.parse_args()
 for name in ['base_profile','ue_root','project','workstation_overlay','output']:setattr(a,name,getattr(a,name).absolute())
 out=a.output;out.mkdir(parents=True,exist_ok=False);runtime=out/'simulator';runtime.mkdir()
 tag=datetime.datetime.now().strftime('%Y%m%d-%H%M%S');names={k:'aerohub-weststreet-'+k+'-'+tag for k in ['px4','onboard','mission']}
 env=prepare(a,out)
 policy={'roles':{'simulator':'host ProjectAirSim/PX4 scene','flight_board':names['px4'],'onboard_computer':names['onboard'],'workstation':names['mission']},'domain':DOMAIN,'system_id':SYSTEM,'instance':INSTANCE,'motion_chain':'workstation ROS Actions -> onboard AS2 Behavior/controller/platform -> PX4 SITL -> ProjectAirSim','height_m':3,'forward_m':5,'max_commanded_speed_mps':1,'arrival_tolerance_m':.75,'stop_speed_mps':.25,'strict_altitude_bounds_m':[-.5,4.5],'ground_only':a.ground_only}
 (out/'predeclared.json').write_text(json.dumps(policy,indent=2));(out/'session.json').write_text(json.dumps(names,indent=2))
 manager=[sys.executable,ROOT/'simulator/manager/manager.py'];started=[];sim_started=False
 def px4_status():return capture(['docker','exec',names['px4'],'/opt/px4-gazebo/bin/px4-listener','--instance',str(INSTANCE),'vehicle_status','-n','1'])
 def docker_base(name):return ['docker','run','-d','--init','--network','host','--ipc','host','--name',name,'--label','aerohub.task=weststreet','-e',f'ROS_DOMAIN_ID={DOMAIN}','-e','ROS_LOCALHOST_ONLY=0','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/repo/deployment/airsim/config/dds/fastdds_udp.xml','-v',str(ROOT)+':/repo:ro','-v',str(out)+':/evidence']
 print('Evidence:',out,flush=True)
 try:
  print('Starting existing simulator manager (PX4 scene, no SimpleFlight)...',flush=True)
  with (out/'manager-start.log').open('w') as f:call(manager+['start','--profile',out/'profile.json','--run-dir',runtime,'--timeout','120'],env=env,stdout=f,stderr=subprocess.STDOUT)
  sim_started=True
  call(docker_base(names['px4'])+['-e','VEHICLE_NAME=Drone1','-e','VEHICLE_NAMESPACE=drone0','-e',f'PX4_INSTANCE={INSTANCE}','-e',f'PX4_SYSTEM_ID={SYSTEM}','-e','XRCE_AGENT_PORT=18991','-e','FLIGHT_BOARD_SENSORS=0','-e','SIMULATOR_RUNTIME_CONFIG=/evidence/simulator/runtime.json','flight_board:v0.1.1','vehicle']);started.append('px4')
  print('Waiting for isolated PX4 actual preflight...',flush=True);deadline=time.monotonic()+100
  while time.monotonic()<deadline:
   try:
    state=px4_status();(out/'px4-preflight.txt').write_text(state)
    if 'pre_flight_checks_pass: True' in state and re.search(r'arming_state:\s*1\b',state):break
   except subprocess.CalledProcessError:pass
   time.sleep(2)
  else:raise RuntimeError('PX4 did not pass preflight; no arming requested')
  call(docker_base(names['onboard'])+['-e','AS2_EXECUTION_BACKEND=follow_path','-e','AS2_FLEET_FILE=/evidence/fleet.yaml','-v',str(ROOT/'deployment/airsim/as2/run.py')+':/opt/onboard_computer/run.py:ro','--entrypoint','bash','onboard_computer:v0.1.1','-lc','source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && source /root/aerostack2_ws/install/setup.bash && source /opt/uav_execution_ws/install/setup.bash && exec python3 /opt/onboard_computer/run.py']);started.append('onboard')
  command='source /opt/ros/humble/setup.bash && source /work/ws/install/setup.bash && exec python3 /repo/deployment/airsim/demos/weststreet/mission.py --output /evidence/mission'
  if a.ground_only:command+=' --ground-only'
  call(docker_base(names['mission'])+['-v',str(a.workstation_overlay)+':/work:ro','--entrypoint','bash','osrf/ros:humble-desktop-full-jammy','-lc',command]);started.append('mission')
  print('Workstation mission started; onboard alone owns AS2 execution.',flush=True)
  deadline=time.monotonic()+240
  while capture(['docker','inspect',names['mission'],'--format','{{.State.Running}}']).strip()=='true':
   if time.monotonic()>deadline:raise TimeoutError('Mission runtime exceeded limit')
   time.sleep(2)
  result_path=out/'mission/result.json'
  if not result_path.is_file():raise RuntimeError('Workstation exited before producing result; inspect mission.log')
  result=json.loads(result_path.read_text());print(json.dumps(result,indent=2))
  if not result['success']:raise RuntimeError('Framework mission incomplete; inspect evidence')
 except KeyboardInterrupt:
  if 'mission' in started:subprocess.run(['docker','kill','--signal','SIGINT',names['mission']],check=False)
  raise
 finally:
  if 'mission' in started and capture(['docker','inspect',names['mission'],'--format','{{.State.Running}}']).strip()=='true':
   subprocess.run(['docker','stop','--timeout','120',names['mission']],check=False)
  safe='px4' not in started
  if not safe:
   try:
    state=px4_status();(out/'px4-final-status.txt').write_text(state);safe=bool(re.search(r'arming_state:\s*1\b',state))
   except Exception as exc:(out/'cleanup-error.txt').write_text(repr(exc))
  for role in started:
   (out/(role+'.log')).write_text(call(['docker','logs',names[role]],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True).stdout)
  if safe:
   for role in reversed(started):subprocess.run(['docker','stop','--timeout','20',names[role]],check=False)
   if sim_started:subprocess.run(manager+['stop','--run-dir',runtime],env=env,stdout=subprocess.DEVNULL)
  else:print('PX4 disarm unconfirmed: retaining simulator and onboard execution for recovery',flush=True)
  (out/'cleanup.json').write_text(json.dumps({'disarm_confirmed':safe,'services_stopped':safe}))

if __name__=='__main__':main()
