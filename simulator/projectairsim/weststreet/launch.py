#!/usr/bin/env python3
"""Run the uncooked street + native ProjectAirSim + existing UAV HUD (no flight)."""
import argparse,json,os,signal,socket,subprocess,time
from pathlib import Path
from urllib.request import ProxyHandler,build_opener

def stop(process):
 if process is None or process.poll() is not None:return
 process.send_signal(signal.SIGINT)
 try:process.wait(timeout=8)
 except subprocess.TimeoutExpired:
  process.terminate()
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:process.kill();process.wait()

def main():
 p=argparse.ArgumentParser(description=__doc__)
 for name in ['ue-root','project','client-python','native-source','output']:p.add_argument('--'+name,type=Path,required=True)
 p.add_argument('--topic-port',type=int,default=19889);p.add_argument('--service-port',type=int,default=19890);p.add_argument('--display-port',type=int,default=19891)
 p.add_argument('--scene-port',type=int,default=19892)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 def interrupted(signum,frame):raise KeyboardInterrupt
 signal.signal(signal.SIGTERM,interrupted)
 if len({a.topic_port,a.service_port,a.display_port,a.scene_port})!=4:raise ValueError('Ports must differ')
 for port in [a.topic_port,a.service_port,a.display_port,a.scene_port]:
  with socket.socket() as s:
   s.settimeout(.3)
   if s.connect_ex(('127.0.0.1',port))==0:raise RuntimeError(f'Port {port} already has a listener')
 engine=a.ue_root.resolve()/'Engine'
 libraries=[engine/'Binaries/Linux',a.project.resolve().parent/'Plugins/ProjectAirSim/Binaries/Linux']
 libraries += [engine/'Plugins'/part/'Binaries/Linux' for part in ['Experimental/ChaosVehiclesPlugin','Media/PixelStreaming','Runtime/ProceduralMeshComponent','Runtime/SunPosition']]
 config=a.output.resolve()/'display.json'
 config.write_text(json.dumps({'enabled':True,'port':a.display_port,'speed_max_mps':20,'global_camera_tag':'WestRunnerGlobalFollow'},indent=2))
 env=os.environ.copy();env['LD_LIBRARY_PATH']=':'.join(map(str,libraries))+(':'+env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
 env['AEROHUB_DISPLAY_CONFIG']=str(config)
 scene_config=a.output.resolve()/'scene-control.json'
 scene_config.write_text(json.dumps({'port':a.scene_port}))
 env['AEROHUB_SCENE_CONTROL_CONFIG']=str(scene_config)
 cmd=[str(engine/'Binaries/Linux/UnrealEditor'),str(a.project.resolve()),'/Game/Aerohub/WestStreetCircuit','-game','-windowed','-ResX=1280','-ResY=720','-nosound','-noraytracing',f'-topicsport={a.topic_port}',f'-servicesport={a.service_port}',f'-abslog={a.output.resolve()/"game.log"}']
 (a.output/'launch-command.json').write_text(json.dumps(cmd,indent=2))
 game=observer=None
 try:
  with (a.output/'game-console.log').open('w') as f:game=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
  opener=build_opener(ProxyHandler({})).open;deadline=time.monotonic()+180
  while True:
   if game.poll() is not None:raise RuntimeError('UE preview exited; inspect game-console.log')
   try:
    with opener(f'http://127.0.0.1:{a.display_port}/aerohub/status',timeout=1) as r:r.read()
    break
   except OSError:
    if time.monotonic()>deadline:raise RuntimeError('HUD readiness timeout')
    time.sleep(.3)
  observer_cmd=[str(a.client_python.absolute()),str(Path(__file__).with_name('observe.py').resolve()),'--native-source',str(a.native_source.resolve()),'--output',str(a.output.resolve()/'session'),'--topic-port',str(a.topic_port),'--service-port',str(a.service_port),'--display-port',str(a.display_port)]
  with (a.output/'observe.log').open('w') as f:observer=subprocess.Popen(observer_cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
  print('Street preview running. Global follows runner; Tab/G select views. No flight commands.',flush=True)
  while game.poll() is None:
   if observer.poll() is not None:raise RuntimeError('Observer exited; inspect observe.log')
   time.sleep(.5)
 finally:
  stop(observer);stop(game)
if __name__=='__main__':
 try:main()
 except KeyboardInterrupt:pass
