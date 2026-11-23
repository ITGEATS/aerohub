#!/usr/bin/env python3
"""Launch the pre-plaza 65s acceptance demo; Ctrl+C requests safe landing."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
from weststreet_ports import PORTS
import re
import signal
import socket
import subprocess
import sys

from runtime_config import DEFAULT_RUNTIME_CONFIG, load_runtime

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
ACTIVE=ROOT/'log/weststreet-before-plaza/active.json'


def owned_process(state):
    try:
        command=Path(f"/proc/{state['child_pid']}/cmdline").read_bytes().split(b'\0')
        words=[x.decode()for x in command if x]
        return ('run_tool_demo.py' in ' '.join(words) and
                'demo_before_plaza.py' in words and state['output'] in words)
    except (OSError,KeyError):
        return False


def check_sources(runtime):
    checkpoint=json.loads((HERE/'before_plaza_checkpoint.json').read_text())
    for name,expected in checkpoint['sha256'].items():
        if hashlib.sha256((HERE/name).read_bytes()).hexdigest()!=expected:
            raise RuntimeError(f'{name} differs from acceptance checkpoint; review before running')
    config=ROOT/'src/basic_function/inference/models/yolo26n_seg/worker.yaml'
    if not re.search(r'^confidence:\s*0\.35\s*$',config.read_text(),re.M):
        raise RuntimeError('Acceptance requires YOLO confidence 0.35')
    manifest=json.loads((runtime/'restored-source.json').read_text())
    for name,expected in manifest['files'].items():
        path=runtime/'src/uav_super_adapter'/name
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise RuntimeError('Restored Navigation source changed: '+name)
    installed=list((runtime/'install/lib').glob('python*/site-packages/uav_super_adapter/navigation.py'))
    expected=manifest['files']['uav_super_adapter/navigation.py']
    if expected!=checkpoint['historical_navigation_sha256']:
        raise RuntimeError('Navigation snapshot is not the historical acceptance version')
    if len(installed)!=1 or hashlib.sha256(installed[0].read_bytes()).hexdigest()!=expected:
        raise RuntimeError('Historical Navigation overlay is missing or differs from checkpoint')
    for name,expected in manifest.get('execution_files',{}).items():
        path=runtime/'src/uav_follow_path_execution'/name
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise RuntimeError('Private execution source changed: '+name)
    expected_binary=manifest.get('execution_binary_sha256')
    if expected_binary:
        binary=runtime/'install/lib/uav_follow_path_execution/follow_path_extended_node'
        if hashlib.sha256(binary.read_bytes()).hexdigest()!=expected_binary:
            raise RuntimeError('Private execution binary differs from checkpoint')
    return checkpoint


def receive_limit(value):
    subprocess.run(['docker','run','--rm','--pull','never','--network','none',
        '--mount','type=bind,src=/proc/sys/net/core/rmem_max,dst=/tmp/aerohub-rmem-max',
        'osrf/ros:humble-desktop-full-jammy','bash','-c',
        'echo '+str(int(value))+' > /tmp/aerohub-rmem-max'],check=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stop',action='store_true',help='Request safe stop of this launcher\'s active run')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--runtime-config', type=Path, default=DEFAULT_RUNTIME_CONFIG,
                        help='Host paths to the scene, UE project, and installed workspaces')
    parser.add_argument('--check-runtime', action='store_true',
                        help='Check runtime paths, checkpoint hashes, and scenario clock without starting')
    parser.add_argument('--legacy-route', action='store_true', help='Use the previously validated out-and-back route')
    parser.add_argument('--fps', type=int, choices=(30, 60), default=30, help='UE window frame rate cap')
    parser.add_argument('--viewer-wait',type=float,default=15.,help='Seconds to prepare screen recording after UE view is ready')
    args=parser.parse_args()
    if args.stop:
        if not ACTIVE.exists():
            print('No active before-plaza run.');return 0
        state=json.loads(ACTIVE.read_text())
        if not owned_process(state):
            print('Recorded run is no longer active.');return 0
        os.killpg(state['child_pid'],signal.SIGINT)
        print('Stop requested; wait for landing and cleanup in the launch terminal.');return 0
    if not 0<=args.viewer_wait<=300:
        parser.error('viewer-wait must be within [0,300]')
    try:
        inputs = load_runtime(args.runtime_config, require_acceptance=True)
        checkpoint = check_sources(inputs['acceptance_workspace'])
        profile = json.loads(inputs['base_profile'].read_text())
        scene_path = inputs['base_profile'].parent / profile['scene_config']
        scene = json.loads(scene_path.read_text())
        clock = scene.get('clock', {})
        if clock.get('step-ns') != 3000000 or clock.get('real-time-update-rate') != 3000000:
            raise RuntimeError('Acceptance requires original 3ms/3ms simulation clock')
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        parser.error(str(error))
    if args.check_runtime:
        print('WestStreet runtime paths, checkpoint hashes and clock verified.')
        return 0
    if ACTIVE.exists() and owned_process(json.loads(ACTIVE.read_text())):
        raise RuntimeError('A before-plaza run is already active; use --stop first')
    # Never replace another task's simulator or worker.
    running=subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines()
    conflicts=[n for n in running if n.startswith('aerohub-west-tools-')]
    if conflicts:raise RuntimeError('Existing WestStreet runtime is active: '+', '.join(conflicts))
    for port in PORTS.values():
        kind=socket.SOCK_DGRAM if port==PORTS['xrce'] else socket.SOCK_STREAM
        with socket.socket(socket.AF_INET,kind)as sock:
            if kind==socket.SOCK_STREAM:sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            try:sock.bind(('127.0.0.1',port))
            except OSError as error:raise RuntimeError(f'Port {port} is in use; existing runtime was left untouched')from error
    stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    output=(args.output or ROOT/'log/weststreet-before-plaza'/stamp).absolute()
    if output.exists():raise RuntimeError('Evidence output already exists: '+str(output))
    output.parent.mkdir(parents=True,exist_ok=True)
    ACTIVE.parent.mkdir(parents=True,exist_ok=True)
    metadata=output.with_name(output.name+'-launcher.json')
    route = 'WestStreetShiftRight2mLate4m' if args.legacy_route else 'WestStreetClosedLoop'
    command=[sys.executable,str(HERE/'run_tool_demo.py'),'--mission-file','demo_before_plaza.py',
        '--runtime-config',str(args.runtime_config.expanduser().resolve()),
        '--component-overlay',str(inputs['acceptance_workspace']),
        '--output',str(output),'--duration','0','--tangent-follow','--tangent-lead','1',
        '--continuous-yaw','--recovery-window','1','--native-audit','--image-storage','png',
        '--ue-map','/Game/Aerohub/'+route,'--yolo-window','--observe-runner',
        '--viewer-wait',str(args.viewer_wait),'--fps',str(args.fps)]
    env=dict(os.environ)
    env.setdefault('DISPLAY',':1')
    env.setdefault('XAUTHORITY',f'/run/user/{os.getuid()}/gdm/Xauthority')
    old=int(Path('/proc/sys/net/core/rmem_max').read_text())
    state=dict(output=str(output),command=command,checkpoint=checkpoint,rmem_before=old)
    child=None
    old_handlers={}
    try:
        if old<4194304:receive_limit(4194304)
        print('Preparing simulator and deployed Tools; takeoff starts automatically when ready.',flush=True)
        print('Record the UE window and YOLO window. UE: Tab = drone view, G = global. Ctrl+C = safe stop.',flush=True)
        print('Evidence: '+str(output),flush=True)
        with output.with_name(output.name+'-launch.log').open('w')as stream:
            child=subprocess.Popen(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            state['child_pid']=child.pid
            ACTIVE.write_text(json.dumps(state,indent=2))
            def stop(_sig,_frame):
                if child.poll() is None:
                    print('Stopping: waiting for landing and cleanup...',flush=True)
                    os.killpg(child.pid,signal.SIGINT)
            for sig in (signal.SIGINT,signal.SIGTERM):old_handlers[sig]=signal.signal(sig,stop)
            code=child.wait()
        state['exit_code']=code
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            child.wait()
        if old<4194304:receive_limit(old)
        state['rmem_restored']=int(Path('/proc/sys/net/core/rmem_max').read_text())
        metadata.write_text(json.dumps(state,indent=2))
        if ACTIVE.exists():
            saved=json.loads(ACTIVE.read_text())
            if saved.get('child_pid')==state.get('child_pid'):ACTIVE.unlink()
        for sig,handler in old_handlers.items():signal.signal(sig,handler)
    result=output/'mission/result.json'
    if result.exists():
        verdict=json.loads(result.read_text())
        print(json.dumps({k:verdict.get(k)for k in ('success','error','cleanup_errors','pursuit_window_complete','landed_disarmed_idle')},indent=2))
    print('Finished; evidence: '+str(output),flush=True)
    return state['exit_code']


if __name__=='__main__':
    raise SystemExit(main())
