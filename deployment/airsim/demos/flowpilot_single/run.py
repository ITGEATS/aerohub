"""Native Linux deployment of the self-contained FlowPilot image.

Scene lifecycle uses the repository AirSim manager; no global process cleanup.
Use --dry-run to inspect exact Docker commands before launching anything.
"""
import argparse
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
DEMO = Path(__file__).resolve().parent
LABEL = 'aerohub.flowpilot.task'


def load_config(path):
    path = path.resolve()
    c = json.loads(path.read_text(encoding='utf-8'))
    required = ('prefix', 'domain', 'image', 'flight_image', 'airsim_host', 'evidence', 'rpc_port', 'goal', 'timeout')
    if not isinstance(c, dict) or any(key not in c for key in required):
        raise ValueError('required config keys: ' + ', '.join(required))
    if set(c) != set(required):
        raise ValueError('unknown configuration keys: ' + ', '.join(sorted(set(c)-set(required))))
    for key in ('prefix', 'image', 'flight_image', 'airsim_host', 'evidence'):
        if not isinstance(c[key], str) or not c[key].strip():
            raise ValueError(key + ' must be a nonempty string')
    if not re.fullmatch(r'[a-z][a-z0-9-]{1,40}', c['prefix']):
        raise ValueError('invalid task prefix')
    if type(c['domain']) is not int or not 0 <= c['domain'] <= 232:
        raise ValueError('invalid DDS domain')
    for key in ('evidence',):
        value = Path(c[key]).expanduser()
        c[key] = str((path.parent / value).resolve() if not value.is_absolute() else value.resolve())
    if c['rpc_port'] != 45471:
        raise ValueError('this packaged single-UAV settings profile uses RPC 45471')
    if c['airsim_host'] not in ('127.0.0.1', 'localhost'):
        raise ValueError('this first delivery profile is single-host Linux; remote AirSim is unsupported')
    import math
    if (not isinstance(c['goal'], list) or len(c['goal']) != 3 or not all(type(x) in (int, float) and math.isfinite(x) for x in c['goal'])
            or type(c['timeout']) not in (int, float) or not math.isfinite(c['timeout']) or c['timeout'] <= 0):
        raise ValueError('finite 3D goal and positive timeout required')
    return c


def run_base(c, name, *, detached=False, gpu=False):
    args = ['docker', 'run', '--init', '--network', 'host', '--name', c['prefix'] + '-' + name,
            '--label', LABEL + '=' + c['prefix'], '-e', 'ROS_DOMAIN_ID=' + str(c['domain'])]
    if detached:
        args += ['-d']
    else:
        args += ['--rm']
    if gpu:
        args += ['--gpus', 'all', '--shm-size', '512m']
    args += ['-e', 'AIRSIM_HOST=' + c['airsim_host'], '-e', 'AIRSIM_RPC_PORT=' + str(c['rpc_port'])]
    return args


def output_user():
    uid, gid = getattr(os, 'getuid', lambda: 1000)(), getattr(os, 'getgid', lambda: 1000)()
    return ['--user', f'{uid}:{gid}', '-e', 'HOME=/tmp', '-e', 'ROS_HOME=/tmp/.ros']


def client(c, case, role, *args):
    # Only independent output data is mounted, never host source or overlays.
    return run_base(c, 'client-' + case) + output_user() + [
        '-v', c['evidence'] + ':/evidence', c['image'], role, *map(str, args)]


def start_commands(c):
    settings = str(DEMO / 'settings.json') + ':/config/settings.json:ro'
    vehicle = run_base(c, 'vehicle', detached=True)
    for k, v in dict(VEHICLE_NAME='PX4', VEHICLE_NAMESPACE='drone0', PX4_INSTANCE='70',
                     PX4_SYSTEM_ID='71', XRCE_AGENT_PORT='8961', FLIGHT_BOARD_SENSORS='0', FLIGHT_BOARD_CAMERAS='0').items():
        vehicle += ['-e', k + '=' + v]
    yield vehicle + ['-v', settings, c['flight_image'], 'vehicle']
    yield run_base(c, 'clock', detached=True) + [c['flight_image'], 'clock']
    yield run_base(c, 'sensor', detached=True) + ['-v', settings, c['image'], 'sensor']
    yield run_base(c, 'overhead', detached=True) + output_user() + ['-v', settings, '-v', c['evidence'] + ':/evidence', c['image'], 'overhead']
    yield run_base(c, 'compute', detached=True, gpu=True) + [c['image'], 'compute']


def require_free_ports():
    # host-net does not isolate ports. Never attach this task to another worker/Agent.
    for kind, port in ((socket.SOCK_STREAM, 18082), (socket.SOCK_DGRAM, 8961)):
        with socket.socket(socket.AF_INET, kind) as probe:
            probe.bind(('0.0.0.0', port))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--case', default='run1')
    parser.add_argument('action', choices=['start', 'status', 'check', 'preflight', 'ground', 'flight', 'cancel-flight',
                                         'pause-depth', 'resume-depth', 'stop', 'reset-round', 'scene-start', 'scene-stop'])
    args = parser.parse_args()
    if not args.dry_run and sys.platform != 'linux':
        parser.error('this delivery entrypoint requires native Linux; use --dry-run to inspect elsewhere')
    c = load_config(args.config)
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.case):
        parser.error('case must be a unique simple identifier')
    evidence = Path(c['evidence'])
    if args.action == 'reset-round':
        for action in ('stop', 'scene-start', 'start'):
            command = [sys.executable, str(Path(__file__).resolve()), '--config', str(args.config.resolve()), action]
            if args.dry_run:
                command.insert(-1, '--dry-run')
                print(json.dumps(command))
            else:
                subprocess.run(command, check=True)
        return
    if args.action.startswith('scene-'):
        env = dict(os.environ, AIRSIM_PROFILE=str(DEMO / 'profile.json'), AIRSIM_RENDER_MODE='windowed',
                   AIRSIM_RUN_DIR=str(evidence / 'scene'))
        command = ['bash', str(ROOT / 'simulator/airsim/airsim.sh'), args.action.split('-')[1]]
        if args.dry_run:
            print(json.dumps({'command': command, 'profile': env['AIRSIM_PROFILE'], 'run_dir': env['AIRSIM_RUN_DIR']}))
        else:
            if args.action == 'scene-start':
                for port in (45471, 4780):
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                        probe.bind(('0.0.0.0', port))
            if args.action == 'scene-stop':
                subprocess.run(client(c, args.case, 'python3', '/repo/deployment/airsim/demos/flowpilot_single/ground_gate.py'), check=True)
            subprocess.run(command, env=env, check=True)
        return
    def execute(command):
        print(json.dumps(command), flush=True)
        if not args.dry_run:
            subprocess.run(command, check=True)
    if args.action == 'start':
        if not args.dry_run:
            if sys.platform != 'linux':
                parser.error('host-net delivery entrypoint requires native Linux')
            subprocess.run(['docker', 'info'], stdout=subprocess.DEVNULL, check=True)
            require_free_ports()
            evidence.mkdir(parents=True, exist_ok=True)
            for role in ('vehicle', 'clock', 'sensor', 'compute', 'overhead'):
                exists = subprocess.run(['docker', 'container', 'inspect', c['prefix'] + '-' + role], capture_output=True)
                if exists.returncode == 0:
                    parser.error('owned-name container exists; inspect before restarting')
        for i, cmd in enumerate(start_commands(c)):
            execute(cmd)
            if i == 0 and not args.dry_run:
                deadline = time.monotonic() + 150
                while time.monotonic() < deadline:
                    logs = subprocess.check_output(['docker', 'logs', c['prefix'] + '-vehicle'], stderr=subprocess.STDOUT, text=True)
                    if 'Ready for takeoff!' in logs:
                        break
                    time.sleep(2)
                else:
                    raise RuntimeError('fresh PX4 failed readiness; retain stack and do not fly')
        if not args.dry_run:
            import urllib.request
            import urllib.error
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            deadline = time.monotonic() + 400
            while time.monotonic() < deadline:
                running = subprocess.check_output(['docker','inspect','--format','{{.State.Running}}',c['prefix']+'-compute'], text=True).strip()
                if running != 'true':
                    raise RuntimeError('compute exited during warmup; retain logs and do not fly')
                try:
                    with opener.open('http://127.0.0.1:18082/health/ready',timeout=2) as response:
                        ready = json.load(response)
                    if (ready.get('model') == 'flowpilot' and ready.get('device') == 'cuda'
                            and ready.get('revision') == 'f29f1bfee1399c96e5fd384276d5d7a09876286b'):
                        break
                except (OSError, ValueError, urllib.error.URLError):
                    pass
                time.sleep(2)
            else:
                raise RuntimeError('CUDA worker warmup timeout; retain stack and do not fly')
            snapshot = evidence / ('stack-' + str(time.time_ns()))
            snapshot.mkdir()
            names = [c['prefix']+'-'+r for r in ('vehicle', 'clock', 'sensor', 'overhead', 'compute')]
            (snapshot/'containers.json').write_bytes(subprocess.check_output(['docker','inspect',*names]))
            (snapshot/'configuration.json').write_text(json.dumps(c,indent=2),encoding='utf-8')
    elif args.action == 'status':
        execute(['docker', 'ps', '-a', '--filter', 'label=' + LABEL + '=' + c['prefix']])
    elif args.action in ('pause-depth', 'resume-depth'):
        name = c['prefix'] + '-sensor'
        if not args.dry_run:
            owner = subprocess.check_output(['docker', 'inspect', '--format', '{{index .Config.Labels "' + LABEL + '"}}', name], text=True).strip()
            if owner != c['prefix']:
                raise RuntimeError('sensor ownership mismatch')
        execute(['docker', 'pause' if args.action == 'pause-depth' else 'unpause', name])
        if not args.dry_run:
            with (evidence / 'injection-events.jsonl').open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(dict(event=args.action, wall=time.time(), monotonic=time.monotonic(), container=name)) + '\n')
    elif args.action == 'stop':
        # Mandatory observed disarmed gate before stopping execution or clock.
        execute(client(c, args.case, 'python3', '/repo/deployment/airsim/demos/flowpilot_single/ground_gate.py'))
        cleanup = evidence / ('cleanup-' + str(time.time_ns()))
        if not args.dry_run:
            cleanup.mkdir(parents=True)
        for role in ('compute', 'sensor', 'clock', 'vehicle', 'overhead'):
            name = c['prefix'] + '-' + role
            if not args.dry_run:
                inspected = subprocess.run(['docker', 'inspect', '--format', '{{index .Config.Labels "' + LABEL + '"}}', name], capture_output=True, text=True)
                if inspected.returncode != 0:
                    continue
                if inspected.stdout.strip() != c['prefix']:
                    raise RuntimeError('ownership mismatch: ' + name)
                (cleanup/(role+'-inspect.json')).write_bytes(subprocess.check_output(['docker','inspect',name]))
                logs = subprocess.run(['docker','logs',name],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
                (cleanup/(role+'.log')).write_bytes(logs.stdout)
                paused = subprocess.check_output(['docker', 'inspect', '--format', '{{.State.Paused}}', name], text=True).strip()
                if paused == 'true':
                    execute(['docker', 'unpause', name])
            execute(['docker', 'stop', '--time', '25', name])
            execute(['docker', 'rm', name])
        scene_command = ['bash', str(ROOT / 'simulator/airsim/airsim.sh'), 'stop']
        if args.dry_run:
            print(json.dumps(scene_command))
        else:
            subprocess.run(scene_command, env=dict(os.environ, AIRSIM_PROFILE=str(DEMO/'profile.json'),
                AIRSIM_RUN_DIR=str(evidence/'scene')), check=True)
    elif args.action == 'check':
        execute(client(c, args.case, 'check'))
    else:
        case = evidence / args.case
        if not args.dry_run:
            case.mkdir(parents=True, exist_ok=False)
        if args.action in ('flight', 'cancel-flight', 'preflight'):
            execute(client(c, args.case, 'preflight', '--idle-s', '15', '--samples', '5',
                           '--goal', *c['goal'], '--output', '/evidence/' + args.case + '/preflight.json'))
        if args.action in ('flight', 'cancel-flight', 'ground'):
            execute(client(c, args.case, 'mission', '--output', '/evidence/' + args.case + '/mission',
                           '--rpc-host', c['airsim_host'], '--rpc-port', c['rpc_port'],
                           '--goal', *c['goal'], '--timeout', c['timeout'], *(['--ground-only'] if args.action == 'ground' else ['--cancel-after-revision', '2'] if args.action == 'cancel-flight' else [])))


if __name__ == '__main__':
    main()
