#!/usr/bin/env python3
"""Owned Mountains moving-target observation; fixed images, no navigation or flight."""
import argparse
import atexit
import json
import os
import shutil
import signal
import shlex
import hashlib
from pathlib import Path
import socket
import subprocess
import sys
import time
from urllib.request import build_opener, ProxyHandler

ROOT=Path(__file__).resolve().parents[4]
APP=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'deployment/airsim/demos/catora_open_field'))
from sim_startup import StartupRelay


def main():
    probe_script = "deployment/target_tracking/airsim/mountains/probe.py"
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scene-package',type=Path,required=True,help='Local Mountains package root; external files are read only')
    parser.add_argument('--settings-file',type=Path,help='Dedicated physical spawn/camera fixture')
    parser.add_argument('--mission-query',default='car')
    parser.add_argument('--estimate-position',action='store_true')
    parser.add_argument('--scene-audit-only',action='store_true',help='Disarmed scene geometry audit; no motion clients')
    parser.add_argument('--flight-only',action='store_true',help='Independent native takeoff/hover/land gate before observation')
    parser.add_argument('--hover-observe',action='store_true',help='Authorized simulated native Takeoff/hover/observe/Land; no pursuit')
    parser.add_argument('--dry-run',action='store_true',help='Print deployment plan without starting resources')
    args=parser.parse_args();output=args.output.resolve()
    if args.dry_run:
        print(json.dumps(dict(backend='grounding_dino',model_id='grounding-dino-tiny',port=8003,
            query=args.mission_query,params='deployment/target_tracking/params.yaml',
            workspace='log/object-detection-query/workspace',worker_start='deployment/inference/start_grounding_dino.sh',
            observation_only=not (args.hover_observe or args.flight_only))))
        return
    if args.flight_only:args.hover_observe=True
    if args.hover_observe:probe_script='deployment/target_tracking/airsim/mountains/hover_probe.py'
    if not output.is_relative_to(ROOT/'log') or output.exists():
        raise ValueError('Use a fresh output directory inside log/')
    for kind,ports in [(socket.SOCK_STREAM,[49471,5180,5190]),(socket.SOCK_DGRAM,[8981,15240,15280])]:
        for port in ports:
            with socket.socket(socket.AF_INET,kind) as probe:probe.bind(('127.0.0.1',port))
    import msgpack
    # Camera audit needs image libraries but never imports a model or starts inference.
    camera_python = os.environ.get('CAMERA_PROBE_PYTHON', sys.executable)
    subprocess.run([camera_python, '-c', 'import cv2, numpy, PIL'], check=True, timeout=10)
    output.mkdir(parents=True)
    shutil.copytree(Path(msgpack.__file__).parent,output/'rpc_deps/msgpack')
    (output/'checks.json').write_text(json.dumps(dict(
        scope='Mountains moving pickup, ground RGB-D/CUDA/Tracking/Mission observation only',
        minimum_unique_tracked_acquisitions=4, minimum_target_displacement_m=1.0,
        mission_duration_s=12, follow_altitude_error_m=1e-9,
        all_follow_geometry_finite=True, disarmed_throughout=True,
        minimum_unique_capture_ids=4, standoff_geometry_error_m=1e-6,
        convoy_stopped_consecutive_samples=5,
        navigation_requests=0, flight_commands=0, convoy_final_speed_max_mps=0.1,
        position_frame='earth', no_truth_to_algorithm=True,
        failures_preserved=True),indent=2)+'\n')
    if args.hover_observe:
        (output/'checks.json').write_text(json.dumps(dict(
            scope='PX4 SITL native AS2 Takeoff/hover, static confirmation then moving observation; no pursuit',
            independent_flight_only=args.flight_only,primary_probe_timeout_s=210,
            px4_preflight_continuous_s=1.,preflight_budget_s=45.,
            relative_takeoff_height_m=3.,takeoff_speed_mps=.5,land_speed_mps=.4,
            hover_height_error_m=.3,hover_xy_drift_m=.5,hover_speed_mps=.2,
            hover_stable_continuous_s=2.,hover_settle_budget_s=15.,
            own_pose_twist_freshness_s=.5,platform_metadata_receive_age_s=1.5,
            parked_confirmation_unique_stamps=4,parked_confirmation_unique_capture_ids=4,
            parked_estimate_spread_m=1.,parked_acquisition_budget_s=12.,
            moving_duration_s=12.,minimum_moving_unique_stamps=4,
            minimum_moving_unique_capture_ids=4,moving_target_displacement_m=1.,
            tracker_validity_s=.5,mission_max_age_s=1.,
            follow_altitude_error_m=1e-9,convoy_stop_speed_mps=.1,
            convoy_stop_consecutive_samples=5,convoy_stop_budget_s=10.,
            native_land_result_required=True,final_disarmed_and_landed_required=True,
            navigation_execute=False,airsim_flight_api=False,truth_algorithm_input=False,
            failure_policy='stop convoy, drain tracking, native Land, confirm ground/disarmed; preserve runtime if unconfirmed'
        ),indent=2)+'\n')
    package=args.scene_package.expanduser().resolve()
    binary=package/'demo/Binaries/Linux/demo-Linux-Shipping'
    assert binary.is_file()
    route_dest=output/'scene/user/Saved/SaveGames'
    route_dest.mkdir(parents=True)
    for name in ('Pickup','Armored','Van'):
        source=package/'demo/Saved/SaveGames'/(name+'Convoy3Route01.sav')
        shutil.copyfile(source,route_dest/source.name)
    (output/'scene-inputs.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [binary, *sorted((package/'demo/Saved/SaveGames').glob('*.sav'))]},indent=2)+'\n')
    worker=None
    worker_log=None
    def stop_worker():
        if worker is not None:
            if worker.poll() is None:
                os.killpg(worker.pid,signal.SIGTERM)
                try:worker.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(worker.pid,signal.SIGKILL);worker.wait(timeout=3)
            if worker_log is not None and not worker_log.closed:worker_log.close()
            (output/'worker-cleanup.json').write_text(json.dumps(dict(pid=worker.pid,exit_code=worker.returncode))+'\n')
    atexit.register(stop_worker)
    try:
        with build_opener(ProxyHandler({})).open('http://127.0.0.1:8003/health/ready',timeout=2) as response:
            health=json.load(response)
    except OSError:
        with socket.socket() as test:
            test.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            test.bind(('127.0.0.1',8003))
        worker_log=(output/'worker.log').open('w')
        worker=subprocess.Popen(['bash',str(ROOT/'deployment/inference/start_grounding_dino.sh')],
            stdout=worker_log,stderr=subprocess.STDOUT,start_new_session=True,
            env={**os.environ,'INFERENCE_PORT':'8003','PYTHONPATH':''})
        (output/'worker-owned.json').write_text(json.dumps(dict(pid=worker.pid,model_id='grounding-dino-tiny',port=8003))+'\n')
        deadline=time.monotonic()+120
        while True:
            try:
                with build_opener(ProxyHandler({})).open('http://127.0.0.1:8003/health/ready',timeout=2) as response:
                    health=json.load(response)
                break
            except OSError:
                if worker.poll() is not None or time.monotonic()>deadline:
                    if worker.poll() is None: os.killpg(worker.pid,signal.SIGTERM)
                    raise RuntimeError('CUDA worker unavailable; see worker.log')
                time.sleep(.3)
    assert health['model_id']=='grounding-dino-tiny' and health['device'].startswith('cuda')
    (output/'worker-health.json').write_text(json.dumps(health,indent=2)+'\n')
    # Pick an unused domain from existing container environments and run records.
    used={192,196}
    for path in (ROOT/'log').glob('**/domain.json'):
        try:used.add(int(json.loads(path.read_text())['domain']))
        except (ValueError,KeyError):pass
    identifiers=subprocess.check_output(['docker','ps','-q'],text=True).split()
    baseline={}
    if identifiers:
        baseline={v['Id']:dict(name=v['Name'],started=v['State']['StartedAt']) for v in json.loads(subprocess.check_output(['docker','inspect',*identifiers],text=True))}
    (output/'existing-containers.json').write_text(json.dumps(baseline,indent=2)+'\n')
    if identifiers:
        for container in json.loads(subprocess.check_output(['docker','inspect',*identifiers],text=True)):
            for entry in container['Config']['Env']:
                if entry.startswith('ROS_DOMAIN_ID='):used.add(int(entry.split('=',1)[1]))
    domain=next(value for value in range(110,192) if value not in used)
    (output/'domain.json').write_text(json.dumps(dict(domain=domain))+'\n')
    settings_source=APP/('hover_settings.json' if args.hover_observe else 'settings.json')
    if args.settings_file:settings_source=args.settings_file.resolve()
    settings=json.loads(settings_source.read_text())
    checks=json.loads((output/'checks.json').read_text())
    checks.update(settings_fixture=str(settings_source.relative_to(ROOT)),mission_query=args.mission_query,
                  detector_backend='grounding_dino',model_id='grounding-dino-tiny',port=8003,
                  box_threshold=.30,text_threshold=.25,inference_timeout_s=10.,snapshot_timeout_s=1.,max_capture_age_s=.5)
    (output/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    # Ground observation and flight use separate physical spawn/camera fixtures.
    (output/'settings.json').write_text(json.dumps(settings,indent=2)+'\n')
    px4=json.loads(json.dumps(settings));px4['Vehicles']['PX4']['TcpPort']=5190
    (output/'px4-settings.json').write_text(json.dumps(px4,indent=2)+'\n')
    profile=json.loads((APP/'profile.json').read_text())
    profile['binary']=str(binary)
    profile['settings']=str(output/'settings.json')
    (output/'profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    (output/'fleet.yaml').write_text('vehicles:\n  drone0:\n    fmu_prefix: /drone0\n    target_system_id: 81\n')
    (output/'transport.json').write_text(json.dumps(dict(network='host',dds='shared fastdds_udp.xml',localhost_only=False,camera_reliable=True))+'\n')
    files=[APP/'profile.json',ROOT/'deployment/airsim/demos/target_tracking/mission.py']+list(APP.glob('*.py'))+[settings_source,ROOT/'deployment/airsim/config/dds/fastdds_udp.xml']+[ROOT/'simulator/airsim/sensors/uav_as2_sensor_adapter/node.py',
        ROOT/'simulator/airsim/sensors/uav_as2_sensor_adapter/conversion.py']
    files += [ROOT/'deployment/airsim/config/as2/config.yaml',ROOT/'deployment/airsim/config/as2/pid_speed_controller.yaml',ROOT/'deployment/airsim/as2/run.py',ROOT/'deployment/airsim/as2/selected_behaviors.launch.py']
    if probe_script is not None:
        files += [ROOT / probe_script] + list((ROOT/'src/tool/target_tracking').rglob('*.py'))
    (output/'sources.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2)+'\n')
    if args.hover_observe:
        for source in files:
            dest=output/'source-snapshot'/source.relative_to(ROOT)
            dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,dest)
    images=['flight_board:v0.1.0','onboard_computer:v0.1.1']
    (output/'images.json').write_text(subprocess.check_output(['docker','image','inspect',*images],text=True))
    prefix=('mountains-hover-' if args.hover_observe else 'mountains-observe-')+str(domain)+'-';owned=[];relay=None;scene_started=False
    relay_process=None;relay_log=None
    common=['--init','--network','host','-e','ROS_DOMAIN_ID='+str(domain),'-e','ROS_LOCALHOST_ONLY=0',
        '-e','FASTDDS_BUILTIN_TRANSPORTS=UDPv4',
        '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/repo/deployment/airsim/config/dds/fastdds_udp.xml',
        '-e','PYTHONDONTWRITEBYTECODE=1','-v',str(ROOT)+':/repo:ro','-v',str(output)+':/evidence',
        '-v',str(ROOT/'log/object-detection-query/workspace')+':/work:ro']
    if args.hover_observe:
        diagnostic=ROOT/'log/yopov2-tracking/px4-diagnostic-overlay'
        if not (diagnostic/'install/setup.bash').exists():raise RuntimeError('Missing verified external PX4 v4 diagnostic overlay')
        common += ['-v',str(diagnostic)+':/px4-overlay:ro']
    def launch(role,image,command,env=()):
        name=prefix+role
        subprocess.run(['docker','run','-d','--name',name,'--label','aerohub.object-detection.output='+str(output),
            *common,*env,'--entrypoint','bash',image,'-lc',command],check=True)
        owned.append(name)
    def manager(action):
        return [sys.executable,str(ROOT/'simulator/airsim/manager/manager.py'),action,
                '--profile',str(output/'profile.json'),'--run-dir',str(output/'scene')]
    try:
        scene_started=True
        subprocess.run(manager('start')+['--timeout','90'],check=True)
        if args.hover_observe:
            relay_log=(output/'relay.log').open('w')
            relay_process=subprocess.Popen([sys.executable,str(APP/'relay.py'),'--ready',str(output/'relay-ready')],
                stdout=relay_log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+5
            while not (output/'relay-ready').exists():
                if relay_process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('owned flight relay unavailable')
                time.sleep(.05)
            (output/'relay-owned.json').write_text(json.dumps(dict(pid=relay_process.pid))+'\n')
        else:relay=StartupRelay(('127.0.0.1',5190),('127.0.0.1',5180))
        launch('vehicle','flight_board:v0.1.0','exec /usr/local/bin/flight-board vehicle',env=[
            '-e','VEHICLE_NAME=PX4','-e','VEHICLE_NAMESPACE=drone0','-e','PX4_INSTANCE=80',
            '-e','PX4_SYSTEM_ID=81','-e','XRCE_AGENT_PORT=8981','-e','AIRSIM_RPC_PORT=49471',
            '-e','SETTINGS_FILE=/evidence/px4-settings.json','-v',str(output/'px4-settings.json')+':/config/settings.json:ro',
            '-e','FLIGHT_BOARD_SENSORS=0','-e','FLIGHT_BOARD_CAMERAS=0'])
        launch('clock','flight_board:v0.1.0','exec /usr/local/bin/flight-board clock',env=[
            '-e','VEHICLE_NAME=PX4','-e','AIRSIM_RPC_PORT=49471'])
        subprocess.run([camera_python,
                        str(APP/'scene_probe.py'),'--output',str(output)],check=True,timeout=30)
        deadline=time.monotonic()+60
        while 'Ready for takeoff!' not in subprocess.check_output(['docker','logs',prefix+'vehicle'],text=True,stderr=subprocess.STDOUT):
            if time.monotonic()>deadline:raise TimeoutError('PX4 ground readiness timeout')
            time.sleep(.25)
        if args.scene_audit_only:return
        launch('sensor','flight_board:v0.1.0',
            'source /opt/ros/humble/setup.bash && source /opt/platform_sensors/install/setup.bash && '
            'export PYTHONPATH=/repo/simulator/airsim/sensors:$PYTHONPATH && '
            'exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args -p settings_file:=/evidence/settings.json '
            '-p rpc_host:=127.0.0.1 -p rpc_port:=49471 -p vehicle_name:=PX4 -p vehicle_namespace:=drone0 '
            '-p source_color_order:=bgr -p camera_rate_hz:=5.0 -p camera_reliable:=true -p use_sim_time:=true -p timing_file:=/evidence/camera-timing.jsonl')
        launch('as2','onboard_computer:v0.1.1',
            'source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && '
            'export AS2_CONFIG_FILE=/repo/deployment/airsim/config/as2/config.yaml && '
            'export AS2_FLEET_FILE=/evidence/fleet.yaml && '+
            ('export AS2_BEHAVIORS=takeoff,land AS2_CONTROLLER_CONFIG_FILE=/repo/deployment/airsim/config/as2/pid_speed_controller.yaml && ' if args.hover_observe else '')+
            'exec python3 /repo/deployment/airsim/as2/run.py')
        command='source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && source /work/install/setup.bash && '
        command+='export PYTHONPATH=/work/python_deps:/evidence/rpc_deps:$PYTHONPATH && export AS2_MODULES_PATH=/work/install/uav_object_detection_python_api/share/uav_object_detection_python_api/modules:/work/install/uav_target_tracking_python_api/share/uav_target_tracking_python_api/modules && '
        if args.hover_observe:command+='source /px4-overlay/install/setup.bash && '
        probe_path = probe_script or 'deployment/object_detection/airsim/probe.py'
        command+='exec python3 /repo/'+probe_path+' --output /evidence/probe'
        if args.flight_only:command+=' --flight-only'
        command+=' --query '+shlex.quote(args.mission_query)
        if args.estimate_position and not args.hover_observe:command+=' --estimate-position'
        owned.append(prefix+'probe')
        with (output/'probe.log').open('w') as probe_log:
            subprocess.run(['docker','run','--rm','--name',prefix+'probe',*common,'--entrypoint','bash','onboard_computer:v0.1.1','-lc',command],
                stdout=probe_log,stderr=subprocess.STDOUT,check=True,timeout=210 if args.hover_observe else 100)
    finally:
        # Independent final host stop guard also covers a crashed/timed-out probe.
        host_stop=dict(confirmed=False,samples=[])
        if scene_started and (output/'scene/state.json').exists():
            sys.path.insert(0,str(ROOT/'simulator/airsim/sensors'))
            from uav_as2_sensor_adapter.rpc import Rpc
            rpc=Rpc('127.0.0.1',49471,2)
            try:
                host_stop['ack']=json.loads(rpc.call('simStopConvoy'))
                deadline=time.monotonic()+10;stable=0
                while time.monotonic()<deadline:
                    state=json.loads(rpc.call('simGetConvoyStatus'))
                    host_stop['samples'].append(state)
                    speeds=[v.get('speed_mps') for v in state.get('vehicles',[])]
                    stable=stable+1 if len(speeds)==3 and all(isinstance(v,(int,float)) and abs(v)<=.1 for v in speeds) else 0
                    if stable>=5:host_stop['confirmed']=True;break
                    time.sleep(.2)
            except Exception as exc:host_stop['error']=repr(exc)
            finally:rpc.close()
        (output/'host-convoy-stop.json').write_text(json.dumps(host_stop,indent=2)+'\n')
        if args.hover_observe and prefix+'as2' in owned:
            running=subprocess.run(['docker','inspect',prefix+'probe'],capture_output=True)
            if running.returncode==0:
                subprocess.run(['docker','kill','--signal','SIGINT',prefix+'probe'],capture_output=True)
                try:subprocess.run(['docker','wait',prefix+'probe'],capture_output=True,timeout=70)
                except subprocess.TimeoutExpired:
                    subprocess.run(['docker','kill',prefix+'probe'],capture_output=True)
            recovery_command=('source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && '
                'source /px4-overlay/install/setup.bash && exec python3 /repo/deployment/target_tracking/airsim/mountains/recover_hover.py --output /evidence/host-recovery')
            with (output/'host-recovery.log').open('w') as recovery_log:
                try:
                    recovery=subprocess.run(['docker','exec',prefix+'as2','bash','-lc',recovery_command],
                        stdout=recovery_log,stderr=subprocess.STDOUT,timeout=100)
                    safe=recovery.returncode==0
                except Exception as exc:safe=False;recovery_log.write(repr(exc))
            if not safe:
                (output/'KEEP_RUNTIME.json').write_text(json.dumps(dict(reason='landing/ground unconfirmed',
                    containers=owned,relay_pid=relay_process.pid if relay_process else None,
                    scene=str(output/'scene'),native_recovery_command=recovery_command),indent=2)+'\n')
                raise RuntimeError('Recovery unconfirmed: flight-support runtime retained; see KEEP_RUNTIME.json')
        for name in reversed(owned):
            if subprocess.run(['docker','inspect',name],capture_output=True).returncode:
                continue
            log=subprocess.run(['docker','logs',name],capture_output=True,text=True)
            (output/(name.removeprefix(prefix)+'.log')).write_text(log.stdout+log.stderr)
            subprocess.run(['docker','stop','--timeout','12',name],check=False)
            subprocess.run(['docker','rm',name],check=False)
        if relay is not None:relay.close()
        if relay_process is not None:
            if relay_process.poll() is None:
                os.killpg(relay_process.pid,signal.SIGTERM);relay_process.wait(timeout=5)
            relay_log.close()
        if scene_started:subprocess.run(manager('stop'),check=False)
        stop_worker()
        remaining=subprocess.check_output(['docker','ps','-a','--filter','name='+prefix,'--format','{{.Names}}'],text=True).split()
        unchanged={}
        for identifier,info in baseline.items():
            inspected=subprocess.run(['docker','inspect',identifier],capture_output=True,text=True)
            actual=json.loads(inspected.stdout)[0] if inspected.returncode==0 else None
            unchanged[identifier]=bool(actual and actual['State']['Running'] and actual['State']['StartedAt']==info['started'])
        (output/'cleanup.json').write_text(json.dumps(dict(remaining_containers=remaining,
            existing_containers_unchanged=unchanged,convoy_stop_confirmed=host_stop['confirmed'],
            worker_owned=worker is not None, worker_exited=worker is None or worker.poll() is not None),indent=2)+'\n')


if __name__=='__main__':
    try:main()
    except Exception as exc:
        import traceback
        if '--output' in sys.argv:
            output=Path(sys.argv[sys.argv.index('--output')+1])
            if output.is_dir():
                (output/'harness-error.json').write_text(json.dumps(dict(error=repr(exc),traceback=traceback.format_exc()),indent=2)+'\n')
        raise
