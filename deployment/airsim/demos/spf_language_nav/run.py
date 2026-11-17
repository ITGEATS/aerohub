#!/usr/bin/env python3
"""SPF language-navigation PC closed-loop demo (WSL2-adapted four-role topology).

Deviations from the Linux-native yopo pattern (recorded in
skill/temp/spf-migration): AirSim (Blocks.exe) runs on the Windows host and is
(re)started through WSL Windows interop instead of simulator/airsim/manager;
PX4 must be the first TCP client after each Blocks restart (single-client
slot). Containers run inside a WSL2 Ubuntu dockerd with --network host.

Run inside WSL:  python3 deployment/airsim/demos/spf_language_nav/run.py <action> ...
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
APP = Path(__file__).resolve().parent
FLIGHT = 'ghcr.io/popc0rn7/aerohub/flight-board:v0.1.1'
ONBOARD = 'ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.1'
GROUND = 'osrf/ros:humble-desktop-full-jammy'
BLOCKS_WIN_PATH = r'D:\Apps\airsim\Blocks\Blocks\WindowsNoEditor'
RPC_PORT = 45481
TCP_PORT = 4581
AGENT_PORT = 8971
PX4_INSTANCE, PX4_SYSTEM_ID = 70, 71


def run(args, **kwargs):
    return subprocess.run(list(map(str, args)), check=True, **kwargs)


def capture(args):
    # Windows interop output may be GBK-encoded (e.g. localized PowerShell
    # errors); decode leniently instead of crashing on non-UTF-8 bytes.
    return subprocess.check_output(list(map(str, args)), text=True,
                                   errors='replace', stderr=subprocess.STDOUT)


def powershell(script):
    # WSL interop is enabled with appendWindowsPath=false, so powershell.exe is
    # not on PATH; fall back to the absolute Windows system path.
    exe = shutil.which('powershell.exe') or '/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe'
    return capture([exe, '-ExecutionPolicy', 'Bypass', '-Command', script])


def windows_host_ip():
    out = capture(['bash', '-c', "ip route show default | awk '{print $3}'"]).strip()
    return out.splitlines()[-1]


def common(output):
    # WSL2 adaptation: ROS_LOCALHOST_ONLY=1 makes FastRTPS use the loopback
    # interface only, and WSL2 does not deliver DDS multicast between the
    # host-network containers over lo (verified: stock talker/listener get 0
    # with =1, 9/10s with =0). The containers share the WSL host netns, so
    # binding to eth0 (NAT'd 172.23.x) keeps them isolated from the LAN while
    # restoring cross-container discovery+data. Deviation recorded in README.
    return ['--init', '--network', 'host',
            '-e', 'ROS_DOMAIN_ID=' + str(domain(output)),
            '-e', 'ROS_LOCALHOST_ONLY=0',
            '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/repo/deployment/airsim/config/dds/fastdds_udp.xml',
            '-e', 'PYTHONDONTWRITEBYTECODE=1',
            '-v', str(ROOT) + ':/repo:ro', '-v', str(output) + ':/evidence']


def domain(output):
    path = output / 'domain.json'
    return json.loads(path.read_text())['domain'] if path.exists() else 190


def prefix(output):
    return 'spf-nav-' + hashlib.sha256(str(output).encode()).hexdigest()[:8] + '-'


def launch(output, role, image, args, env=(), mounts=(), entrypoint=None):
    command = ['docker', 'run', '-d', '--name', prefix(output) + role,
               '--label', 'aerohub.spf.output=' + str(output),
               *common(output), *env, *mounts]
    if entrypoint:
        command += ['--entrypoint', entrypoint]
    command += [image, *args]
    run(command)


def win_path(p):
    """Convert a WSL path to the Windows form; powershell.exe cannot resolve
    /mnt/c paths (and the repository path contains non-ASCII characters)."""
    return capture(['wslpath', '-w', str(p)]).strip()


def restart_blocks(output, observe=False):
    """Blocks must be fresh so PX4 is the first TCP client (single-slot server)."""
    settings = json.loads((APP / 'settings.json').read_text())
    if observe:
        # M7 (AGENTS multi-deployment rule): enable the centroid-tracking
        # top-down view through the shared deployment/swarm infrastructure.
        sys.path.insert(0, str(ROOT / 'deployment' / 'swarm'))
        from view import apply_overhead_view
        settings = apply_overhead_view(settings)
    (output / 'settings.json').write_text(json.dumps(settings, indent=2) + '\n')
    win_settings = win_path(output / 'settings.json')
    powershell(
        "$s = Get-Content -Raw '" + win_settings + "'; "
        "if (-not $s) { throw 'settings.json read failed' } "
        "$dir = Join-Path $env:USERPROFILE 'Documents\\AirSim'; "
        "New-Item -ItemType Directory -Force -Path $dir | Out-Null; "
        "if (Test-Path (Join-Path $dir 'settings.json')) { Copy-Item (Join-Path $dir 'settings.json') (Join-Path $dir ('settings.backup.' + (Get-Date -Format 'yyyyMMddHHmmss') + '.json')) }; "
        "Set-Content -Path (Join-Path $dir 'settings.json') -Value $s -Encoding utf8; "
        "taskkill /F /IM Blocks.exe 2>$null; Start-Sleep 4")
    # Launch Blocks through a Windows scheduled task, NOT a direct Start-Process.
    # A process started from WSL interop lives in the interop job object and is
    # killed when the `wsl ... run.py start` command returns (verified: Blocks
    # died ~15s after start with 3GB free and no crash dump). A task created for
    # the logged-on user runs under Task Scheduler, detached from that job, and
    # survives for the whole mission. The window resolution MUST be >= the FPV
    # camera capture resolution (1280x720): a smaller window makes UE4
    # reallocate the scene render target every frame (1024x640 <-> 1280x720
    # thrash), pegging the CPU and destabilising the lockstep mavlink link.
    powershell(
        "schtasks /delete /tn SPFBlocks /f 2>$null | Out-Null; "
        "$tr = '" + BLOCKS_WIN_PATH + "\\\\Blocks.exe -NoSplash -windowed -ResX=1280 -ResY=720 -nosound'; "
        "schtasks /create /tn SPFBlocks /tr $tr /sc once /st 00:00 /f | Out-Null; "
        "schtasks /run /tn SPFBlocks")
    deadline = time.time() + 90
    while True:
        listening = powershell(
            "netstat -ano | Select-String ':%d.*LISTENING'" % TCP_PORT)
        if listening.strip():
            break
        if time.time() > deadline:
            raise RuntimeError('Blocks TCP %d never listened' % TCP_PORT)
        time.sleep(2)


def pick_domain(output):
    used = {json.loads(p.read_text())['domain']
            for p in (ROOT / 'log/spf/pc-e2e').rglob('domain.json')} if (ROOT / 'log/spf/pc-e2e').exists() else set()
    identifiers = capture(['docker', 'ps', '-q']).split()
    if identifiers:
        for container in json.loads(capture(['docker', 'inspect', *identifiers])):
            for entry in container['Config']['Env']:
                if entry.startswith('ROS_DOMAIN_ID='):
                    used.add(int(entry.split('=', 1)[1]))
    available = [v for v in range(180, 200) if v not in used]
    if not available:
        raise RuntimeError('no unused DDS domain')
    (output / 'domain.json').write_text(json.dumps({'domain': available[0]}) + '\n')


def vlm_api_key():
    api_key_file = Path.home() / '.spf_vlm_api_key'
    api_key = api_key_file.read_text().strip() if api_key_file.exists() else os.environ.get('OPENAI_API_KEY', '')
    if not api_key:
        raise RuntimeError('set OPENAI_API_KEY or provide ~/.spf_vlm_api_key before start')
    return api_key


def launch_compute(output):
    launch(output, 'compute', ONBOARD,
           ['-lc', 'exec bash /repo/deployment/airsim/demos/spf_language_nav/spf_compute.sh'],
           env=['-e', 'SPF_FLEET_FILE=/evidence/fleet.yaml'], entrypoint='bash')


def launch_proxy(output):
    launch(output, 'proxy', GROUND, ['-lc',
           'source /opt/ros/humble/setup.bash && '
           'source /repo/log/spf/pc-e2e/ros_ws/install/setup.bash && '
           'exec ros2 run uav_spf_vlm_proxy spf_vlm_proxy --ros-args -r __ns:=/drone0 '
           '--params-file /repo/deployment/airsim/demos/spf_language_nav/planner.yaml'],
           env=['-e', 'OPENAI_API_KEY=' + vlm_api_key()], entrypoint='bash')


def check_owner(output, role):
    name = prefix(output) + role
    label = capture(['docker', 'inspect', '--format',
                     '{{index .Config.Labels "aerohub.spf.output"}}', name]).strip()
    if label != str(output):
        raise RuntimeError('resource owner mismatch')
    return name


def restart_compute(output):
    """Re-create the compute + proxy containers after an overlay rebuild.

    Both roles source the ros_ws overlay, so a topic/interface change requires
    recreating them together; the vehicle/clock/sensor containers are unaffected."""
    for role in ('proxy', 'compute'):
        run(['docker', 'rm', '-f', check_owner(output, role)])
    launch_compute(output)
    launch_proxy(output)


def start(output, observe=False):
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'images.json').exists():
        raise RuntimeError('use a fresh output directory')
    winip = windows_host_ip()
    (output / 'topology.json').write_text(json.dumps(
        {'windows_host_ip': winip, 'rpc_port': RPC_PORT, 'tcp_port': TCP_PORT,
         'blocks_path': BLOCKS_WIN_PATH, 'overhead_view': bool(observe)},
        indent=2) + '\n')
    pick_domain(output)
    for image in [FLIGHT, ONBOARD, GROUND]:
        run(['docker', 'image', 'inspect', image], stdout=subprocess.DEVNULL)
    (output / 'images.json').write_text(json.dumps(
        {i: capture(['docker', 'image', 'inspect', i, '--format', '{{.Id}}']).strip()
         for i in [FLIGHT, ONBOARD, GROUND]}, indent=2) + '\n')
    (output / 'fleet.yaml').write_text(
        'vehicles:\n  drone0:\n    fmu_prefix: /drone0\n    target_system_id: %d\n' % PX4_SYSTEM_ID)
    source_manifest = {'files': {}, 'spf_upstream': capture(
        ['git', '-C', ROOT / 'src/planner/spf/upstream', 'rev-parse', 'HEAD']).strip()}
    for folder in [APP, ROOT / 'src/planner/spf/ros2_adapter', ROOT / 'src/planner/spf/vlm_proxy',
                   ROOT / 'src/planner/interfaces', ROOT / 'src/behavior/follow_path']:
        for path in folder.rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts and '.pytest_cache' not in path.parts:
                source_manifest['files'][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (output / 'source-manifest.json').write_text(json.dumps(source_manifest, indent=2) + '\n')

    restart_blocks(output, observe)
    launch(output, 'vehicle', FLIGHT, ['vehicle'],
           env=['-e', 'VEHICLE_NAME=PX4', '-e', 'VEHICLE_NAMESPACE=drone0',
                '-e', 'PX4_INSTANCE=%d' % PX4_INSTANCE, '-e', 'PX4_SYSTEM_ID=%d' % PX4_SYSTEM_ID,
                '-e', 'XRCE_AGENT_PORT=%d' % AGENT_PORT,
                '-e', 'AIRSIM_HOST=' + winip, '-e', 'AIRSIM_RPC_PORT=%d' % RPC_PORT,
                '-e', 'FLIGHT_BOARD_SENSORS=0', '-e', 'FLIGHT_BOARD_CAMERAS=0'],
           mounts=['-v', str(output / 'settings.json') + ':/config/settings.json:ro'])
    launch(output, 'clock', FLIGHT, ['clock'],
           env=['-e', 'VEHICLE_NAME=PX4', '-e', 'AIRSIM_HOST=' + winip,
                '-e', 'AIRSIM_RPC_PORT=%d' % RPC_PORT],
           mounts=['-v', str(output / 'settings.json') + ':/config/settings.json:ro'])
    deadline = time.monotonic() + 90
    while 'Ready for takeoff!' not in capture(['docker', 'logs', prefix(output) + 'vehicle']):
        if time.monotonic() > deadline:
            raise RuntimeError('PX4 preflight timeout; preserve resources for diagnosis')
        time.sleep(.5)
    launch(output, 'sensor', FLIGHT, ['-lc',
           'source /opt/ros/humble/setup.bash && source /opt/platform_sensors/install/setup.bash && '
           'exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args '
           '-p settings_file:=/config/settings.json -p rpc_host:=' + winip + ' -p rpc_port:=%d ' % RPC_PORT +
           '-p vehicle_name:=PX4 -p vehicle_namespace:=drone0 -p camera_rate_hz:=10.0 -p use_sim_time:=true'],
           mounts=['-v', str(output / 'settings.json') + ':/config/settings.json:ro'], entrypoint='bash')
    launch_compute(output)
    launch_proxy(output)


def observer_command(output, case, winip, once):
    cmd = ['docker', 'run']
    if once:
        cmd += ['--rm']
    else:
        cmd += ['-d', '--name', prefix(output) + 'observer',
                '--label', 'aerohub.spf.output=' + str(output)]
    cmd += common(output) + [
        '-e', 'PYTHONPATH=/repo/simulator/airsim/sensors:/repo/deployment/robopi/vendor',
        '--entrypoint', 'bash', GROUND, '-lc',
        ('exec ' if not once else '') +
        'python3 /repo/deployment/airsim/demos/spf_language_nav/spf_observer.py '
        '--settings /evidence/settings.json --host ' + winip +
        ' --evidence /evidence/' + case + (' --once' if once else '')]
    return cmd


def mission(output, args):
    case_path = output / args.case
    if case_path.exists():
        raise RuntimeError('use a fresh case name; evidence must not be overwritten')
    topology = json.loads((output / 'topology.json').read_text())
    winip = topology['windows_host_ip']
    observe = bool(topology.get('overhead_view'))
    command = ['docker', 'run', '--rm', *common(output),
               '-e', 'PYTHONPATH=/repo/simulator/airsim/sensors:/repo/deployment/robopi/vendor',
               '--entrypoint', 'bash', GROUND, '-lc',
               'source /opt/ros/humble/setup.bash && source /repo/log/spf/pc-e2e/workstation/ws/install/setup.bash && '
               'exec python3 /repo/deployment/airsim/demos/spf_language_nav/mission.py '
               '--output /evidence/' + args.case + ' --instruction "' + args.instruction +
               '" --max-cycles ' + str(args.max_cycles) + ' --timeout ' + str(args.timeout) +
               ' --rpc-host ' + winip + ' --rpc-port %d --fault ' % RPC_PORT + args.fault +
               ' --target ' + ' '.join(str(v) for v in args.target) +
               ' --spawn ' + ' '.join(str(v) for v in args.spawn)]
    observer = None
    try:
        if observe:
            # Health check: one-shot update proves the external camera exists
            # and is placed over the CURRENT centroid. Per the AGENTS rule this
            # must never be reported as tracking success: the proof is the
            # moving centroid series in <case>/overhead/track.jsonl recorded
            # while the mission flies.
            health = capture(observer_command(output, args.case, winip, True))
            (output / (args.case + '.observer-health.json')).write_text(health)
            run(observer_command(output, args.case, winip, False))
            observer = prefix(output) + 'observer'
        if args.fault not in ('stale', 'vlm-down'):
            run(command)
            return
        case_path.mkdir(parents=True, exist_ok=True)
        request = case_path / 'fault-request.json'
        process = subprocess.Popen(list(map(str, command)))
        paused = stopped = None
        injected = False
        try:
            while process.poll() is None:
                if request.exists() and not injected:
                    evidence = {'fault': args.fault, 'wall': time.time()}
                    if args.fault == 'stale':
                        container = prefix(output) + 'sensor'
                        run(['docker', 'pause', container])
                        paused = container
                        evidence['sensor_container'] = container
                    else:
                        container = prefix(output) + 'proxy'
                        run(['docker', 'stop', '--time', '5', container])
                        stopped = container
                        evidence['proxy_container'] = container
                    (case_path / 'fault-injection.json').write_text(json.dumps(evidence, indent=2) + '\n')
                    injected = True
                time.sleep(.1)
            if not injected:
                raise RuntimeError('task ended before fault injection')
        finally:
            if paused:
                run(['docker', 'unpause', paused])
                (case_path / 'sensor-restored.json').write_text(json.dumps({'wall': time.time()}) + '\n')
    finally:
        if observer:
            try:
                (case_path / 'observer.log').write_text(capture(['docker', 'logs', observer]))
            finally:
                run(['docker', 'stop', '--timeout', '5', observer])
                run(['docker', 'rm', observer])


def stop(output):
    results = list(output.glob('*/result.json'))
    latest = json.loads(max(results, key=lambda p: p.stat().st_mtime).read_text()) if results else {}
    # Cleanup is safe when the drone is confirmed disarmed, or when the latest
    # task never reached flight (failure at readiness/teleport/takeoff leaves
    # the drone on the ground; run y formal1 had no disarmed key because it
    # never armed). Anything from navigation onward must confirm disarm.
    error = str(latest.get('error', ''))
    safe_on_ground = any('at ' + stage + ':' in error
                         for stage in ('readiness', 'teleport', 'takeoff'))
    # A start-only session (no task ever ran, e.g. observer health failure)
    # leaves the drone on the ground and is always safe to clean up.
    if results and not (latest.get('disarmed') or safe_on_ground):
        raise RuntimeError('latest task must confirm disarmed before cleanup')
    observer = prefix(output) + 'observer'
    if capture(['docker', 'ps', '-aq', '--filter', 'name=' + observer]).strip():
        run(['docker', 'rm', '-f', observer])
    for role in ['proxy', 'compute', 'sensor', 'clock', 'vehicle']:
        name = prefix(output) + role
        label = capture(['docker', 'inspect', '--format', '{{index .Config.Labels "aerohub.spf.output"}}', name]).strip()
        if label != str(output):
            raise RuntimeError('resource owner mismatch')
        (output / (role + '.txt')).write_text(capture(['docker', 'logs', name]))
        run(['docker', 'stop', '--timeout', '15', name])
        run(['docker', 'rm', name])
    powershell('taskkill /F /IM Blocks.exe 2>$null; '
               'schtasks /delete /tn SPFBlocks /f 2>$null | Out-Null')


def prepare(output):
    """Build spf packages + interfaces into the shared ros_ws overlay (run inside WSL).

    The repository is bind-mounted at /repo inside the container; all in-container
    paths must use the mount point, never the WSL host path (Chinese characters in
    the host path also break docker -w resolution)."""
    work = '/repo/log/spf/pc-e2e/ros_ws'
    (ROOT / 'log/spf/pc-e2e/ros_ws').mkdir(parents=True, exist_ok=True)
    run(['docker', 'run', '--rm', '-v', str(ROOT) + ':/repo', '-w', work,
         '--entrypoint', 'bash', ONBOARD, '-lc',
         'source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && '
         'cd ' + work + ' && colcon build --base-paths '
         '/repo/src/planner/interfaces /repo/src/planner/spf/ros2_adapter /repo/src/planner/spf/vlm_proxy '
         '/repo/src/behavior/follow_path '
         '--packages-select uav_planner_interfaces uav_spf_planner uav_spf_vlm_proxy '
         'uav_follow_path_execution '
         '--event-handlers console_direct+'])
    prepare_ground()


def prepare_ground():
    """Build the canonical as2_msgs subset that mission.py consumes (yopo pattern).

    The workstation image has no Aerostack2; only the interfaces the task actually
    uses are generated in its own environment, pinned by image IDs and file hashes."""
    ws = ROOT / 'log/spf/pc-e2e/workstation/ws'
    source = ws / 'src/as2_msgs'
    source.mkdir(parents=True, exist_ok=True)
    manifest = {}
    for name in ['msg/PlatformInfo.msg', 'msg/PlatformStatus.msg', 'msg/ControlMode.msg',
                 'action/Takeoff.action', 'action/Land.action']:
        text = capture(['docker', 'run', '--rm', '--network', 'none', '--entrypoint', 'cat',
                        ONBOARD, '/root/aerostack2_ws/src/aerostack2/as2_msgs/' + name])
        path = source / name
        path.parent.mkdir(exist_ok=True)
        path.write_text(text)
        manifest[name] = hashlib.sha256(text.encode()).hexdigest()
    (source / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.8)
project(as2_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/PlatformInfo.msg" "msg/PlatformStatus.msg"
  "msg/ControlMode.msg" "action/Takeoff.action" "action/Land.action" DEPENDENCIES std_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
''')
    (source / 'package.xml').write_text('''<package format="3"><name>as2_msgs</name><version>1.1.3</version>
<description>Canonical task interface subset from onboard image</description>
<maintainer email="dev@example.com">Aerohub</maintainer><license>BSD-3-Clause</license>
<buildtool_depend>ament_cmake</buildtool_depend><build_depend>rosidl_default_generators</build_depend>
<depend>std_msgs</depend><depend>action_msgs</depend><exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group><export><build_type>ament_cmake</build_type></export></package>''')
    manifest['ground_image'] = capture(['docker', 'image', 'inspect', GROUND, '--format', '{{.Id}}']).strip()
    manifest['onboard_image'] = capture(['docker', 'image', 'inspect', ONBOARD, '--format', '{{.Id}}']).strip()
    (ws / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    run(['docker', 'run', '--rm', '--network', 'none', '-v', str(ROOT) + ':/repo',
         '-w', '/repo/log/spf/pc-e2e/workstation/ws', '--entrypoint', 'bash', GROUND, '-lc',
         'source /opt/ros/humble/setup.bash && colcon build --event-handlers console_direct+'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'start', 'mission', 'stop', 'restart-compute'])
    parser.add_argument('--output', type=Path, default=None)
    parser.add_argument('--case', default='ground')
    parser.add_argument('--instruction', default='fly toward the orange ball')
    parser.add_argument('--max-cycles', type=int, default=8)
    parser.add_argument('--timeout', type=float, default=420.)
    parser.add_argument('--fault', choices=['none', 'cancel', 'stale', 'vlm-down'], default='none')
    parser.add_argument('--target', type=float, nargs=3, default=[34.25, 33.10, -3.68])
    parser.add_argument('--spawn', type=float, nargs=3, default=[10.0, 10.0, 45.0],
                        help='teleport spawn x y yaw_deg (default matches mission.py)')
    parser.add_argument('--observe', action='store_true',
                        help='start action only: enable the centroid-tracking '
                             'overhead view (M7); missions then run an observer '
                             'container and record overhead evidence')
    args = parser.parse_args()
    if args.output is None:
        raise ValueError('--output required (inside log/spf/pc-e2e)')
    output = args.output.resolve()
    if not str(output).startswith(str(ROOT / 'log')):
        raise ValueError('output must be inside repository log/')
    if args.action == 'prepare':
        prepare(output)
    elif args.action == 'start':
        start(output, args.observe)
    elif args.action == 'mission':
        mission(output, args)
    elif args.action == 'stop':
        stop(output)
    elif args.action == 'restart-compute':
        restart_compute(output)


if __name__ == '__main__':
    main()
