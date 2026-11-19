#!/usr/bin/env python3
"""Own a bounded, disarmed AirSim/PX4 camera validation; no flight commands."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from urllib.request import build_opener, ProxyHandler

ROOT=Path(__file__).resolve().parents[3]
APP=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'deployment/airsim/demos/catora_open_field'))
from sim_startup import StartupRelay


def main(probe_script=None, extra_workspace=None, detector_backend="yolo"):
    if detector_backend not in ("yolo", "grounding_dino"):
        raise ValueError("unknown detector backend")
    dino = detector_backend == "grounding_dino"
    port = 8003 if dino else 8002
    model_id = "grounding-dino-tiny" if dino else "yolo26n-seg"
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--estimate-position',action='store_true')
    parser.add_argument('--samples',type=int,default=10)
    parser.add_argument('--collect-failures',action='store_true')
    parser.add_argument('--near-target',action='store_true',help='Require valid near car measurements and save scoring inputs')
    parser.add_argument('--vehicle-x',type=float,default=0.)
    parser.add_argument('--vehicle-y',type=float,default=0.)
    parser.add_argument('--vehicle-yaw',type=float,default=0.)
    parser.add_argument('--dry-run',action='store_true',help='Print deployment plan without starting resources')
    args=parser.parse_args();output=args.output.resolve()
    if args.dry_run:
        print(json.dumps(dict(backend=detector_backend,model_id=model_id,port=port,query='car',
            params='deployment/target_tracking/params.yaml' if dino else 'deployment/object_detection/params.example.yaml',
            workspace='log/object-detection-query/workspace' if dino else 'log/object-detection-tool/workspace',
            worker_owned=False,observation_only=True)))
        return
    if not output.is_relative_to(ROOT/'log') or output.exists():
        raise ValueError('Use a fresh output directory inside log/')
    for kind,ports in [(socket.SOCK_STREAM,[49471,5180,5190]),(socket.SOCK_DGRAM,[8981,15240,15280])]:
        for port in ports:
            with socket.socket(socket.AF_INET,kind) as probe:probe.bind(('127.0.0.1',port))
    with build_opener(ProxyHandler({})).open(f'http://127.0.0.1:{port}/health/ready',timeout=5) as response:
        health=json.load(response)
    assert health['model_id']==model_id and health['device'].startswith('cuda')
    camera_python = os.environ.get('CAMERA_PROBE_PYTHON', sys.executable) if dino else str(ROOT/'src/basic_function/inference/models/yolo26n_seg/.venv/bin/python')
    if dino:
        subprocess.run([camera_python, '-c', 'import cv2, numpy'], check=True, timeout=10)
    output.mkdir(parents=True)
    # Pick an unused domain from existing container environments and run records.
    used={192,196}
    for path in (ROOT/'log').glob('**/domain.json'):
        try:used.add(int(json.loads(path.read_text())['domain']))
        except (ValueError,KeyError):pass
    identifiers=subprocess.check_output(['docker','ps','-q'],text=True).split()
    if identifiers:
        for container in json.loads(subprocess.check_output(['docker','inspect',*identifiers],text=True)):
            for entry in container['Config']['Env']:
                if entry.startswith('ROS_DOMAIN_ID='):used.add(int(entry.split('=',1)[1]))
    domain=next(value for value in range(110,171) if value not in used)
    (output/'domain.json').write_text(json.dumps(dict(domain=domain))+'\n')
    settings=json.loads((APP/'settings.json').read_text())
    settings['Vehicles']['PX4'].update(X=args.vehicle_x,Y=args.vehicle_y,Yaw=args.vehicle_yaw)
    if not args.estimate_position:
        camera=settings['Vehicles']['PX4']['Cameras']['FrontCamera']
        camera['CaptureSettings']=[c for c in camera['CaptureSettings'] if c['ImageType']==0]
    (output/'settings.json').write_text(json.dumps(settings,indent=2)+'\n')
    px4=json.loads(json.dumps(settings));px4['Vehicles']['PX4']['TcpPort']=5190
    (output/'px4-settings.json').write_text(json.dumps(px4,indent=2)+'\n')
    profile=json.loads((ROOT/'simulator/airsim/scenarios/catora_open_field/profile.json').read_text())
    profile['settings']=str(output/'settings.json')
    (output/'profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    (output/'fleet.yaml').write_text('vehicles:\n  drone0:\n    fmu_prefix: /drone0\n    target_system_id: 81\n')
    (output/'transport.json').write_text(json.dumps(dict(network='host',dds='shared fastdds_udp.xml',localhost_only=False,camera_reliable=True))+'\n')
    checks=['disarmed','real RGB8 640x480','unique camera stamps',
            'new frame through public Module','actual CUDA','success includes observed no detections']
    if args.near_target:
        if not args.estimate_position:raise ValueError('--near-target requires --estimate-position')
        checks += ['ten consecutive car positions at 2-8m','independent scene collision depth error <=0.30m','output transform residual <=0.05m']
    if probe_script is not None:
        checks = ['disarmed before and after', 'real AirSim RGB-D through actual CUDA detector',
                  'at least four confirmed TrackObject feedbacks consumed via AS2 Module',
                  'finite capture stamps and covariance', 'standard action cancel',
                  'static ground only; moving target, accuracy, faults and navigation unfinished']
    checks += (['real depth and matching calibration','RGB-depth skew <=50ms','capture-time AS2 TF']
               if args.estimate_position else ['no depth capture configured','no depth/calibration/TF consumers in Tool'])
    (output/'checks.json').write_text(json.dumps(dict(checks=checks,position_requested=args.estimate_position,
        limitation='No flight, forced target insertion or position accuracy claim'),indent=2)+'\n')
    import hashlib
    files=list(APP.glob('*.py'))+[APP/'settings.json',ROOT/'deployment/airsim/config/dds/fastdds_udp.xml']+[ROOT/'simulator/airsim/sensors/uav_as2_sensor_adapter/node.py',
        ROOT/'simulator/airsim/sensors/uav_as2_sensor_adapter/conversion.py']
    if probe_script is not None:
        files += [ROOT / probe_script] + list((ROOT/'src/tool/target_tracking').rglob('*.py'))
    (output/'sources.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2)+'\n')
    compute_image='onboard_computer:v0.1.1' if dino else 'onboard_computer:v0.1.0'
    images=['flight_board:v0.1.0',compute_image]
    (output/'images.json').write_text(subprocess.check_output(['docker','image','inspect',*images],text=True))
    prefix='object-detection-rgb-'+str(domain)+'-';owned=[];relay=None;scene_started=False
    common=['--init','--network','host','-e','ROS_DOMAIN_ID='+str(domain),'-e','ROS_LOCALHOST_ONLY=0',
        '-e','FASTDDS_BUILTIN_TRANSPORTS=UDPv4',
        '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/repo/deployment/airsim/config/dds/fastdds_udp.xml',
        '-e','PYTHONDONTWRITEBYTECODE=1','-v',str(ROOT)+':/repo:ro','-v',str(output)+':/evidence',
        '-v',str(ROOT/('log/object-detection-query/workspace' if dino else 'log/object-detection-tool/workspace'))+':/work:ro']
    if extra_workspace is not None:
        common += ['-v', str(extra_workspace)+':/tracking:ro']
    def launch(role,image,command,env=()):
        name=prefix+role
        subprocess.run(['docker','run','-d','--name',name,'--label','aerohub.object-detection.output='+str(output),
            *common,*env,'--entrypoint','bash',image,'-lc',command],check=True)
        owned.append(name)
    def manager(action):
        return [sys.executable,str(ROOT/'simulator/manager/manager.py'),action,
                '--profile',str(output/'profile.json'),'--run-dir',str(output/'scene')]
    try:
        scene_started=True
        subprocess.run(manager('start')+['--timeout','90'],check=True)
        relay=StartupRelay(('127.0.0.1',5190),('127.0.0.1',5180))
        launch('vehicle','flight_board:v0.1.0','exec /usr/local/bin/flight-board vehicle',env=[
            '-e','VEHICLE_NAME=PX4','-e','VEHICLE_NAMESPACE=drone0','-e','PX4_INSTANCE=80',
            '-e','PX4_SYSTEM_ID=81','-e','XRCE_AGENT_PORT=8981','-e','AIRSIM_RPC_PORT=49471',
            '-e','SETTINGS_FILE=/evidence/px4-settings.json','-v',str(output/'px4-settings.json')+':/config/settings.json:ro',
            '-e','FLIGHT_BOARD_SENSORS=0','-e','FLIGHT_BOARD_CAMERAS=0'])
        launch('clock','flight_board:v0.1.0','exec /usr/local/bin/flight-board clock',env=[
            '-e','VEHICLE_NAME=PX4','-e','AIRSIM_RPC_PORT=49471'])
        deadline=time.monotonic()+60
        while 'Ready for takeoff!' not in subprocess.check_output(['docker','logs',prefix+'vehicle'],text=True,stderr=subprocess.STDOUT):
            if time.monotonic()>deadline:raise TimeoutError('PX4 ground readiness timeout')
            time.sleep(.25)
        subprocess.run([camera_python,
                        str(APP/'scene_reference.py'),'--output',str(output)],check=True,timeout=30)
        launch('sensor','flight_board:v0.1.0',
            'source /opt/ros/humble/setup.bash && source /opt/platform_sensors/install/setup.bash && '
            'export PYTHONPATH=/repo/simulator/airsim/sensors:$PYTHONPATH && '
            'exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args -p settings_file:=/evidence/settings.json '
            '-p rpc_host:=127.0.0.1 -p rpc_port:=49471 -p vehicle_name:=PX4 -p vehicle_namespace:=drone0 '
            '-p source_color_order:=bgr -p camera_rate_hz:=5.0 -p camera_reliable:=true -p use_sim_time:=true -p timing_file:=/evidence/camera-timing.jsonl')
        launch('as2',compute_image,
            'source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && '
            'export AS2_CONFIG_FILE=/repo/deployment/airsim/config/as2/config.yaml && '
            'export AS2_FLEET_FILE=/evidence/fleet.yaml && exec python3 /repo/deployment/airsim/as2/run.py')
        command='source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && source /work/install/setup.bash && '
        command+='export PYTHONPATH=/work/python_deps:$PYTHONPATH && '
        modules = ('/work/install/uav_object_detection_python_api/share/uav_object_detection_python_api/modules:/work/install/uav_target_tracking_python_api/share/uav_target_tracking_python_api/modules'
                   if dino else '/work/install/share/uav_object_detection_python_api/modules')
        command+='export AS2_MODULES_PATH='+modules+' && '
        if extra_workspace is not None:
            command+='source /tracking/install/setup.bash && export AS2_MODULES_PATH=/tracking/install/uav_target_tracking_python_api/share/uav_target_tracking_python_api/modules:$AS2_MODULES_PATH && '
        probe_path = probe_script or 'deployment/object_detection/airsim/probe.py'
        command+='exec python3 /repo/'+probe_path+' --output /evidence/probe'
        if args.estimate_position:command+=' --estimate-position'
        if args.collect_failures:command+=' --collect-failures'
        if args.near_target:command+=' --near-target --samples '+str(args.samples)
        owned.append(prefix+'probe')
        subprocess.run(['docker','run','--rm','--name',prefix+'probe',*common,'--entrypoint','bash',('onboard_computer:v0.1.1' if dino else 'onboard_computer:v0.1.0'),'-lc',command],check=True,timeout=100)
        if args.near_target:
            subprocess.run([camera_python,
                            str(APP/'score_near.py'),'--output',str(output),'--samples',str(args.samples)],check=True,timeout=60)
    finally:
        for name in reversed(owned):
            if subprocess.run(['docker','inspect',name],capture_output=True).returncode:
                continue
            log=subprocess.run(['docker','logs',name],capture_output=True,text=True)
            (output/(name.removeprefix(prefix)+'.log')).write_text(log.stdout+log.stderr)
            subprocess.run(['docker','stop','--timeout','12',name],check=False)
            subprocess.run(['docker','rm',name],check=False)
        if relay is not None:relay.close()
        if scene_started:subprocess.run(manager('stop'),check=False)


if __name__=='__main__':main()
