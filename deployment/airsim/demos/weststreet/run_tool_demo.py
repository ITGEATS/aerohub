#!/usr/bin/env python3
"""Host deployment/scene fixture for the Tool-only WestStreet mission."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import shutil
import socket
import subprocess
import sys
import time
from urllib.request import build_opener, ProxyHandler

from run import prepare, ROOT, DOMAIN, INSTANCE, SYSTEM
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient
from simulator.projectairsim.display_session import views_from_scene


def check_navigation_overlay(workspace, build_command):
    package = 'uav_planner_interfaces'
    source = ROOT / 'src/tool/navigation/interfaces'
    install = workspace / 'install'
    installed = install / 'share' / package
    if not installed.is_dir():
        installed = install / package / 'share' / package
    for directory, suffix in (('action', '.action'), ('msg', '.msg'), ('srv', '.srv')):
        for definition in sorted((source / directory).glob('*' + suffix)):
            generated = installed / directory / definition.name
            if not generated.is_file() or generated.read_bytes() != definition.read_bytes():
                raise RuntimeError(
                    f'Navigation overlay {workspace} needs a rebuild ({directory}/{definition.name}); '
                    f'run {build_command}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ue-map',choices=['/Game/Aerohub/WestStreetCircuit','/Game/Aerohub/WestStreetSmoothTurn','/Game/Aerohub/WestStreetShiftRight2m', '/Game/Aerohub/WestStreetShiftRight2mLate4m','/Game/Aerohub/WestStreetClosedLoop'],default='/Game/Aerohub/WestStreetShiftRight2mLate4m',help='Current tracking baseline: 4m turns, 2.75m/s; other maps require explicit selection')
    parser.add_argument('--mission-file',choices=['demo.py','demo_before_plaza.py'],default='demo_before_plaza.py')
    parser.add_argument('--component-overlay',type=Path,help='Optional isolated component overlay; leaves shared installs untouched')
    parser.add_argument('--duration', type=float, default=5.)
    parser.add_argument('--runner-start-delay',type=float,default=0.,help='Diagnostic quiet-target delay after acquisition, seconds')
    parser.add_argument('--system-only', action='store_true')
    parser.add_argument('--tangent-lead',type=float,default=.35)
    parser.add_argument('--tangent-follow',action='store_true')
    parser.add_argument('--predictive-observation',action='store_true')
    parser.add_argument('--continuous-yaw',action='store_true',help='Executor-local continuous yaw, same navigation owner')
    parser.add_argument('--target-yaw',action='store_true',help='Enable optional SUPER target-yaw capability; caller must supply TARGET_YAW')
    parser.add_argument('--native-audit',action='store_true',help='Transparent single-peer topic relay with native image header probe')
    parser.add_argument('--observe-runner',action='store_true',help='Record independent actor poses for turn scoring only')
    parser.add_argument('--image-storage',choices=['raw','png'],default='raw')
    parser.add_argument('--yolo-window',action='store_true',help='Owned official YOLO window, closed on task exit even with keep-scene')
    parser.add_argument('--recovery-window',type=float,default=.5,help='Explicit bounded target prediction window, seconds (.5 to 2)')
    parser.add_argument('--keep-scene',action='store_true',help='Keep owned simulator/PX4 disarmed for viewing after the run')
    parser.add_argument('--fps', type=int, choices=(30, 60), default=30, help='UE window frame rate cap')
    parser.add_argument('--viewer-wait',type=float,default=0.,help='Pause before mission startup so the live window can be inspected')
    parser.add_argument('--reuse-runtime', type=Path, help='Resume an owned disarmed scene after startup failure')
    args = parser.parse_args()
    if not 0 <= args.tangent_lead <= 1.5:
        parser.error('tangent-lead must be between 0 and 1.5 seconds')
    if not .5 <= args.recovery_window <= 2.:
        parser.error('recovery-window must be between .5 and 2 seconds')
    unlimited = not args.system_only and args.duration == 0
    if not (args.runner_start_delay >= 0 and (unlimited or args.runner_start_delay < args.duration)):
        parser.error('runner-start-delay must be nonnegative and below mission duration')
    if not args.system_only:
        receive_limit = int(Path('/proc/sys/net/core/rmem_max').read_text())
        if receive_limit < 4194304:
            parser.error('Image DDS requires net.core.rmem_max >= 4194304 before starting receivers; '
                         'see deployment/airsim/config/dds/README.md')
    else:
        receive_limit = None
    if not 0 <= args.viewer_wait <= 300:
        parser.error('viewer-wait must be between 0 and 300 seconds')
    if args.target_yaw and args.continuous_yaw:
        parser.error('Select one yaw policy')
    workspaces = ROOT / 'log/weststreet-demo'
    nav_workspace = workspaces / 'super-workspace'
    client_workspace = workspaces / 'client-workspace'
    check_navigation_overlay(
        nav_workspace, 'SUPER_APP_IMAGE=uav/apps-super-forest:onboard-v0.1.1 bash '
        'deployment/airsim/demos/super_forest/build_navigation_overlay.sh '
        'log/weststreet-demo/super-workspace')
    check_navigation_overlay(
        client_workspace, 'bash deployment/airsim/demos/weststreet/build_demo_overlay.sh')
    out = args.output.absolute()
    out.mkdir(parents=True, exist_ok=False)
    if args.component_overlay:
        args.component_overlay=args.component_overlay.resolve()
        if not (args.component_overlay/'install/local_setup.bash').is_file():
            raise RuntimeError('Component overlay has not been built')
        source_manifest=args.component_overlay/'restored-source.json'
        if source_manifest.exists():shutil.copyfile(source_manifest,out/'component-source.json')
    args.base_profile = ROOT/'log/projectairsim/package-course-wmm/profile.json'
    args.ue_root = Path('/home/wqy/UE_installed_build')
    args.project = args.ue_root/'WestStreet/PAirSim.uproject'
    args.perception_only = True
    env = prepare(args, out)
    profile_path = out/'profile.json'
    profile = json.loads(profile_path.read_text())
    profile['args'][1]=args.ue_map
    profile['args'].append(f'-ExecCmds=t.MaxFPS {args.fps}')
    camera = profile['sensor_options']['Drone1']['FrontCamera']
    camera.update(width=640, height=480, captures=['rgb'], rotation=[0.,0.,0.], position=[.6,0.,.8])
    base = json.loads(args.base_profile.read_text())
    profile['sensor_options']['Drone1']['lidar1'] = base['sensor_options']['Drone1']['lidar1']
    # Native FRD z>0 is below the body: mount above, not inside the road.
    profile['sensor_options']['Drone1']['lidar1']['position'] = [0.,0.,-.2]
    robot_path = out/'robot_px4.jsonc'
    robot = json.loads(robot_path.read_text())
    for sensor in robot['sensors']:
        if sensor.get('type')!='camera':continue
        sensor['enabled']=sensor['id']=='FrontCamera'
        sensor['capture-interval']=1./15.
        for capture in sensor.get('capture-settings',[]):
            capture['capture-enabled']=sensor['id']=='FrontCamera' and capture['image-type']==0
        if sensor['id']=='FrontCamera':
            sensor['gimbal']={'lock-roll':True,'lock-pitch':True,'lock-yaw':False}
            sensor['origin']={'xyz':'0.6 0 0.8','rpy-deg':'0 0 0'}
            sensor['aerohub-pointing']=False

    lidar = dict(id='lidar1', type='lidar', enabled=True)
    lidar.update({'parent-link':'Frame','number-of-channels':16,'range':100.,
        'points-per-second':100000,'horizontal-rotation-frequency':10.,
        'horizontal-fov-start-deg':0.,'horizontal-fov-end-deg':360.,
        'vertical-fov-upper-deg':15.,'vertical-fov-lower-deg':-15.,
        'draw-debug-points':False,'origin':{'xyz':'0 0 -0.2','rpy-deg':'0 0 0'}})
    robot['sensors'] = [sensor for sensor in robot['sensors'] if sensor['id']!='lidar1']+[lidar]
    robot_path.write_text(json.dumps(robot,indent=2))
    if args.native_audit:
        profile['adapter']['environment']['PROJECTAIRSIM_TOPIC_RELAY_PORT']='20189'
    profile['adapter']['environment']['AEROHUB_VISION_TIMING']='1'
    profile['adapter']['environment']['PROJECTAIRSIM_TRACKING_OVERLAY']=str(ROOT/'log/target-tracking-follow/workspace')
    profile_path.write_text(json.dumps(profile, indent=2))
    runtime = out/'simulator'
    runtime.mkdir()
    if args.reuse_runtime:
        runtime = args.reuse_runtime.absolute()/'simulator'
    tag = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    names = {role:'aerohub-west-tools-'+role+'-'+tag for role in
             ('px4','onboard','tools','navigation','mission','bag')}
    if args.reuse_runtime:
        old = json.loads((args.reuse_runtime/'session.json').read_text())
        if not old['px4'].startswith('aerohub-west-tools-px4-'):
            raise RuntimeError('Not an owned WestStreet Tool runtime')
        names['px4'] = old['px4']
    (out/'session.json').write_text(json.dumps(names, indent=2))
    manifest = dict(source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        host_rmem_max=receive_limit, domain=DOMAIN, instance=INSTANCE, system_id=SYSTEM, scene=args.ue_map,
        duration_wall_s=args.duration, runner_start_delay_s=args.runner_start_delay, person_speed_mps=5., height_m=2.,
        camera=dict(position_frd_m=[.6,0.,.8],active_pointing=False,width=640,height=480,fov_deg=90,pitch_deg=0.,gimbal="lock-roll/pitch; yaw follows body",capture_hz=15.,captures=['rgb'],height_prior_m=1.7),
        tracking=dict(position_filter='none',measurement_stddev_m=1.,association_policy='soft_innovation_v1',blind_timeout_s=args.recovery_window,distance_prior_m=3.,reachable_distance_m=8.),
        component_overlay=str(args.component_overlay) if args.component_overlay else None, mission_file=args.mission_file, tangent_lead_s=args.tangent_lead, tangent_follow=args.tangent_follow, predictive_observation=args.predictive_observation, image_storage=args.image_storage, recovery_window_s=args.recovery_window, yolo_window=args.yolo_window, continuous_yaw_enabled=args.continuous_yaw, target_yaw_enabled=args.target_yaw, keep_scene=args.keep_scene, viewer_wait_s=args.viewer_wait, independent_runner_observation=args.observe_runner,
        topology='host simulator; PX4 container; onboard AS2/Tools/SUPER containers; workstation client',
        scope='System real flight' if args.system_only else 'restored Tool-only pursuit; full-loop acceptance recorded separately',
        gates=dict(tool_mission_success=True, physical_disarm=True,
                   runner_start_after_target_acquired=True, actual_navigation_displacement_m=1.,
                   collision_coverage='not certified; retain available native contact evidence'),
        not_covered=['full loop','turning pursuit','obstacle clearance guarantee','repeated reliability'])
    (out/'predeclared.json').write_text(json.dumps(manifest, indent=2))
    (out/'source.diff').write_text(subprocess.check_output(['git','diff'],cwd=ROOT,text=True))
    # Capture untracked authored runtime/mission too; git diff alone omits them.
    tracked_files = list(Path(__file__).parent.glob('*.py'))
    (out/'source-sha256.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked_files},indent=2))
    actor = ProjectAirSimExtensionClient(port=19891,scene_port=19892)
    actor_running = False
    next_actor_observation = 0.
    runner_ready_at = None
    native_probe=None
    native_probe_stream=None
    viewer = viewer_stream = None
    worker = None
    worker_stream = None
    started = []
    sim_attempted = False
    manager = [sys.executable,str(ROOT/'simulator/manager/manager.py')]
    def run(command, **kw):
        return subprocess.run(list(map(str, command)),check=True,**kw)
    def capture(command):
        return run(command,capture_output=True,text=True).stdout
    def px4_status():
        return capture(['docker','exec',names['px4'],'/opt/px4-gazebo/bin/px4-listener',
                        '--instance',str(INSTANCE),'vehicle_status','-n','1'])
    def base_docker(role, image, command, extra=()):
        mounts = {ROOT:'/repo', out:'/evidence',
            nav_workspace:'/nav',
            client_workspace:'/clients',
            ROOT/'log/system-tool/workspace':'/system',
            ROOT/'log/object-detection-query/workspace':'/query',
            ROOT/'log/target-tracking-follow/workspace':'/tracking',
            ROOT/'log/weststreet-demo/perception-workspace':'/perception',
            ROOT/'log/projectairsim/weststreet-framework/workstation':'/work'}
        if args.component_overlay:mounts[args.component_overlay]='/acceptance'
        cmd = ['docker','run','-d','--init','--network','host','--ipc','host',
            '--name',names[role],'--label','aerohub.task=weststreet-tool-demo',
            '-e',f'ROS_DOMAIN_ID={DOMAIN}','-e','ROS_LOCALHOST_ONLY=0',
            '-e','PYTHONDONTWRITEBYTECODE=1','-e','AEROHUB_MOTION_LOCK_DIR=/evidence/motion',
            '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/repo/deployment/airsim/config/dds/fastdds_udp.xml']
        for host, target in mounts.items():
            cmd += ['-v',str(host)+':'+target+('' if target=='/evidence' else ':ro')]
        cmd += list(extra)+['--entrypoint','bash',image,'-lc',command]
        run(cmd)
        started.append(role)
    def wait(predicate, timeout, description):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>end:
                raise TimeoutError(description)
            time.sleep(.25)
    onboard_setup = ('source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && '
        'source /root/aerostack2_ws/install/setup.bash && source /opt/uav_execution_ws/install/setup.bash && ')
    tool_setup = (onboard_setup+'source /query/install/setup.bash && source /tracking/install/setup.bash && '
        'source /system/install/setup.bash && source /perception/install/setup.bash && source /nav/install/local_setup.bash && '
        'export PYTHONPATH=/query/python_deps:/repo${PYTHONPATH:+:$PYTHONPATH} && ')
    super_setup = (onboard_setup+'source /opt/super_ws/install/setup.bash && source /opt/super_app_ws/install/setup.bash && '
        'source /nav/install/setup.bash && export PYTHONPATH=/nav/python_deps${PYTHONPATH:+:$PYTHONPATH} && ')
    component_setup='source /acceptance/install/local_setup.bash && ' if args.component_overlay else ''
    super_setup+=component_setup
    print('Evidence: '+str(out),flush=True)
    try:
        for port in (() if args.reuse_runtime else (19889,19890,19891,19892,18002,18991)+((20189,) if args.native_audit else ())):
            deadline = time.monotonic()+70
            while True:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as check:
                        check.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                        check.bind(('127.0.0.1',port))
                    break
                except OSError:
                    if time.monotonic()>deadline:
                        raise RuntimeError('Port still unavailable: '+str(port))
                    time.sleep(1)
        if not args.system_only:
            model = ROOT/'src/basic_function/inference/models/yolo26n_seg'
            worker_stream = (out/'yolo-worker.log').open('w')
            worker = subprocess.Popen([str(model/'.venv/bin/python'),'-m','uvicorn','serve:app',
                '--host','127.0.0.1','--port','18002','--workers','1'],cwd=model,
                env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',HF_HUB_OFFLINE='1'),
                stdout=worker_stream,stderr=subprocess.STDOUT)
            opener = build_opener(ProxyHandler({}))
            def gpu_ready():
                if worker.poll() is not None:
                    raise RuntimeError('YOLO worker exited')
                try:
                    with opener.open('http://127.0.0.1:18002/health/ready',timeout=2) as response:
                        health = json.load(response)
                    (out/'gpu-health.json').write_text(json.dumps(health,indent=2))
                    return health.get('device','').startswith('cuda')
                except OSError:
                    return False
            wait(gpu_ready,90,'YOLO GPU readiness')
        if args.native_audit:
            native_probe_stream=(out/'native-probe.log').open('w')
            probe_env=dict(env)
            probe_env['PYTHONPATH']=profile['client_path']+os.pathsep+probe_env.get('PYTHONPATH','')
            native_probe=subprocess.Popen([profile['client_python'],str(ROOT/'simulator/projectairsim/native_topic_relay.py'),
                '--output',str(out/'native-frames.jsonl')],env=probe_env,stdout=native_probe_stream,stderr=subprocess.STDOUT)
            wait(lambda:(out/'native-frames.jsonl.ready').exists(),10,'native relay startup')
        if not args.reuse_runtime:
            sim_attempted = True
            with (out/'manager-start.log').open('w') as stream:
                run(manager+['start','--profile',profile_path,'--run-dir',runtime,'--timeout','120'],
                    env=env,stdout=stream,stderr=subprocess.STDOUT)
            # Scene lifecycle is simulator-side, never in the workstation demo.
            (out/'runner-reset.json').write_text(json.dumps(actor.reset_actor()))
            px4_command = ['docker','run','-d','--init','--network','host','--ipc','host','--name',names['px4'],
                '-e',f'ROS_DOMAIN_ID={DOMAIN}','-e','VEHICLE_NAME=Drone1','-e','VEHICLE_NAMESPACE=drone0',
                '-e',f'PX4_INSTANCE={INSTANCE}','-e',f'PX4_SYSTEM_ID={SYSTEM}','-e','XRCE_AGENT_PORT=18991',
                '-e','FLIGHT_BOARD_SENSORS=0','-e','SIMULATOR_RUNTIME_CONFIG=/evidence/simulator/runtime.json',
                '-v',str(out)+':/evidence','flight_board:v0.1.1','vehicle']
            run(px4_command); started.append('px4')
        else:
            sim_attempted = True
            started.append('px4')
        def preflight():
            try:
                state = px4_status()
            except subprocess.CalledProcessError:
                return False
            (out/'px4-preflight.txt').write_text(state)
            return 'pre_flight_checks_pass: True' in state and bool(re.search(r'arming_state:\s*1\b',state))
        wait(preflight,100,'PX4 preflight')
        views,_=views_from_scene(json.loads((runtime/'resolved_scene.json').read_text()),include_captured_views=True)
        views=[v for v in views if v['vehicle_id']=='Drone1' and v['camera_id']=='FrontCamera' and v['image_type']=='RGB']
        if not views:raise RuntimeError('No enabled FrontCamera RGB view')
        actor.call('RegisterViews',views=views)
        if not actor.set_view('Drone1','FrontCamera','RGB'):
            raise RuntimeError('FrontCamera view selection was rejected')
        if args.mission_file in ('demo_before_plaza.py','demo.py'):
            actor.set_title('请让无人机保持8米距离追踪白衣男子', side='left')
            actor.set_title('5 m/s 奔跑', side='right')
        view=actor.get_state()
        (out/'viewer-ready.json').write_text(json.dumps(view,indent=2))
        if not view.get('view_available'):raise RuntimeError('FrontCamera component is not available for viewing')
        print('Viewer ready: click UE window, Tab = drone camera, G = global',flush=True)
        if args.viewer_wait:time.sleep(args.viewer_wait)
        base_docker('onboard','onboard_computer:v0.1.1',onboard_setup+
            'source /nav/install/setup.bash && '+component_setup+'exec python3 /repo/deployment/airsim/as2/run.py',
            ['-e','AS2_EXECUTION_BACKEND=follow_path','-e','AS2_FLEET_FILE=/evidence/fleet.yaml'])
        tool_command = (tool_setup+'exec ros2 run uav_system_tool system_tool --ros-args -r __ns:=/drone0 -p use_sim_time:=true'
            if args.system_only else tool_setup+f'exec python3 /repo/deployment/airsim/demos/weststreet/tool_runtime.py --output /evidence/detections --recovery-window {args.recovery_window} --image-storage {args.image_storage}')
        base_docker('tools','onboard_computer:v0.1.1',tool_command)
        if args.yolo_window and not args.system_only:
            wait(lambda:(out/'detections/calls.jsonl').exists(),60,'YOLO viewer input readiness')
            viewer_stream=(out/'yolo-viewer.log').open('w')
            viewer=subprocess.Popen([profile['client_python'],str(ROOT/'deployment/airsim/demos/weststreet/yolo_viewer.py'),
                str(out),'--exit-on-idle','5'],env=env,stdout=viewer_stream,stderr=subprocess.STDOUT,start_new_session=True)
            (out/'viewer-process.json').write_text(json.dumps(dict(pid=viewer.pid,owned=True)))
        if not args.system_only:
            base_docker('navigation','uav/apps-super-forest:onboard-v0.1.1',super_setup+
                'exec python3 /repo/deployment/airsim/demos/weststreet/navigation_runtime.py --output /evidence/navigation'+(' --continuous-yaw' if args.continuous_yaw else (' --target-yaw' if args.target_yaw else '')))
            wait(lambda:(out/'navigation/ready.json').exists(),120,'SUPER actual lidar map readiness')
        base_docker('bag','uav/apps-super-forest:onboard-v0.1.1',super_setup+
            'exec ros2 bag record -o /evidence/flight-bag /clock /tf /tf_static '
            '/drone0/platform/info /drone0/self_localization/pose /drone0/self_localization/twist '
            '/drone0/sensor_measurements/odom /drone0/motion_reference/trajectory '
            '/drone0/navigation/odom /drone0/super_adapter/state /planning_cmd/poly_traj '
            '/drone0/tools/target_tracking/state /drone0/gimbal/status '
            '/drone0/tools/target_tracking/measurement_filter /diagnostics/vision_pipeline /diagnostics/vision_udp')
        mission_setup = ('source /opt/ros/humble/setup.bash && source /work/ws/install/setup.bash && '
            'export DEMO_CLIENT_OVERLAY=/clients/install && ')
        if args.system_only:
            mission_command = (mission_setup+'source /clients/install/setup.bash && '
                'exec python3 /repo/deployment/system/demo.py --output /evidence/mission')
        else:
            mission_command = (mission_setup+f'exec env WESTSTREET_MISSION_FILE={args.mission_file} bash /repo/deployment/airsim/demos/weststreet/run_demo.sh '
                f'--output /evidence/mission --duration {args.duration} --recovery-window {args.recovery_window}'+(' --predictive-observation' if args.predictive_observation else '')+(f' --tangent-follow --tangent-lead {args.tangent_lead}' if args.tangent_follow else '')+(' --continuous-yaw' if args.continuous_yaw else (' --target-yaw' if args.target_yaw else '')))
        base_docker('mission','osrf/ros:humble-desktop-full-jammy',mission_command)
        end = float('inf') if unlimited else time.monotonic()+max(220., args.duration+180.)
        while capture(['docker','inspect',names['mission'],'--format','{{.State.Running}}']).strip()=='true':
            if shutil.disk_usage(out).free<128*1024*1024:
                (out/'evidence-stop').touch()
                run(['docker','kill','--signal','SIGINT',names['mission']])
                raise RuntimeError('Evidence disk budget exhausted; task canceled, no further image writes')
            if time.monotonic()>end:
                run(['docker','kill','--signal','SIGINT',names['mission']])
                raise TimeoutError('Mission deadline')
            if args.observe_runner and time.monotonic()>=next_actor_observation:
                next_actor_observation=time.monotonic()+.2
                try:
                    sample=dict(wall_time=time.time(),state=actor.get_actor_state())
                except Exception as error:
                    sample=dict(wall_time=time.time(),error=str(error))
                with (out/'runner-observation.jsonl').open('a')as stream:stream.write(json.dumps(sample)+'\n')
            events_file = out/'mission/events.json'
            if not args.system_only and not actor_running and events_file.exists():
                events = json.loads(events_file.read_text())
                if any(event['event']=='target_acquired' for event in events) and runner_ready_at is None:
                    runner_ready_at = time.monotonic()+args.runner_start_delay
                if runner_ready_at is not None and time.monotonic()>=runner_ready_at:
                    (out/'runner-start.json').write_text(json.dumps(actor.start_actor()))
                    actor_running = True
                    print('Runner started after takeoff and target acquisition',flush=True)
            time.sleep(.1)
        result_file = out/'mission/result.json'
        if not result_file.exists():
            raise RuntimeError('Mission produced no result; inspect mission.log')
        result = json.loads(result_file.read_text())
        print(json.dumps(result,indent=2),flush=True)
        if not result['success']:
            raise RuntimeError('Real Tool mission failed')
        if not args.system_only and not actor_running:
            raise RuntimeError('Runner was never started')
    finally:
        # Observation lifetime is independent of retained simulator resources.
        if viewer is not None:
            viewer.terminate()
            try:viewer.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(viewer.pid,signal.SIGKILL);viewer.wait(timeout=5)
        if viewer_stream is not None:viewer_stream.close()
        if actor_running:
            try:
                (out/'runner-stop.json').write_text(json.dumps(actor.stop_actor()))
            except Exception as error:
                (out/'runner-stop-error.txt').write_text(str(error))
        if 'mission' in started:
            running = capture(['docker','inspect',names['mission'],'--format','{{.State.Running}}']).strip()
            if running=='true':
                run(['docker','kill','--signal','SIGINT',names['mission']])
                wait(lambda:capture(['docker','inspect',names['mission'],'--format','{{.State.Running}}']).strip()!='true',130,'Mission cleanup')
        safe = 'px4' not in started
        if not safe:
            try:
                state = px4_status()
                (out/'px4-final-status.txt').write_text(state)
                safe = bool(re.search(r'arming_state:\s*1\b',state))
            except Exception as error:
                (out/'cleanup-error.txt').write_text(str(error))
        if worker is not None:
            worker.terminate()
            try:
                worker.wait(timeout=10)
            except subprocess.TimeoutExpired:
                worker.kill(); worker.wait(timeout=5)
        if worker_stream is not None:
            worker_stream.close()
        if safe:
            for role in reversed(started):
                if args.keep_scene and role=='px4':continue
                run(['docker','stop','--timeout','20',names[role]])
            if sim_attempted and not args.keep_scene:
                run(manager+['stop','--run-dir',runtime],env=env,stdout=subprocess.DEVNULL)
        else:
            print('Physical disarm unconfirmed; retaining simulator/onboard for recovery',flush=True)
        for role in started:
            logs = subprocess.run(['docker','logs',names[role]],capture_output=True,text=True)
            (out/(role+'.log')).write_text(logs.stdout+logs.stderr)
            (out/(role+'-inspect.json')).write_text(capture(['docker','inspect',names[role]]))
        if native_probe is not None and safe and not args.keep_scene:
            native_probe.terminate()
            try:native_probe.wait(timeout=10)
            except subprocess.TimeoutExpired:
                native_probe.kill();native_probe.wait(timeout=5)
        if native_probe_stream is not None:native_probe_stream.close()
        if safe and args.keep_scene:
            (out/'retained-runtime.json').write_text(json.dumps(dict(runtime=str(runtime),px4=names['px4'],
                native_relay_pid=native_probe.pid if native_probe is not None else None,
                scene_port=19892,display_port=19891,disarm_confirmed=True),indent=2))
        (out/'cleanup.json').write_text(json.dumps(dict(disarm_confirmed=safe,services_stopped=safe and not args.keep_scene,
            execution_services_stopped=safe,scene_retained=safe and args.keep_scene,
            retained_native_relay_pid=native_probe.pid if native_probe is not None and (not safe or args.keep_scene) else None)))


if __name__=='__main__':
    main()
