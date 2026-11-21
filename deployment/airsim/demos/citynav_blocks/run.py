#!/usr/bin/env python3
"""CityNav language-navigation PC closed-loop demo (Windows host + Docker Desktop).

Four-role topology adapted from the validated spf_language_nav skeleton
(skill/temp/citynav-migration records the deviations). Differences from the
WSL-based SPF run.py:
  - everything runs natively on the Windows host: docker CLI talks to the
    Docker Desktop engine; Blocks.exe is (re)started through a Windows
    scheduled task (a direct Start-Process GUI child is reaped by the job
    object that wraps spawned processes - same conclusion as SPF);
  - containers reach the Windows host (Blocks RPC/TCP + GPU inference
    services) through an address picked at start time by host_probe.py
    (host.docker.internal or a vEthernet IPv4; recorded in topology.json);
  - planner and Inference load separate generated parameter files;
    host worker addresses are rendered into <output>/inference.yaml.

PX4 must be the first TCP client after each Blocks restart (single-client
slot); every case therefore runs start -> mission -> stop with a fresh Blocks.

Usage (PowerShell, repository root):
  python deployment/airsim/demos/citynav_blocks/run.py prepare --output log/citynav/pc-e2e/ros_ws_stub
  python deployment/airsim/demos/citynav_blocks/run.py start   --output log/citynav/pc-e2e/run-XXXX [--observe]
  python deployment/airsim/demos/citynav_blocks/run.py health  --output log/citynav/pc-e2e/run-XXXX
  python deployment/airsim/demos/citynav_blocks/run.py mission --output log/citynav/pc-e2e/run-XXXX --case formal1
  python deployment/airsim/demos/citynav_blocks/run.py stop    --output log/citynav/pc-e2e/run-XXXX
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
APP = Path(__file__).resolve().parent
FLIGHT = 'ghcr.io/popc0rn7/aerohub/flight-board:v0.1.1'
ONBOARD = 'ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.1'
GROUND = 'osrf/ros:humble-desktop-full-jammy'
# Local machine paths are NEVER committed (contrib guide 6.2): point the
# Blocks WindowsNoEditor directory at your own install through this
# environment variable, e.g.
#   $env:CITYNAV_BLOCKS_PATH = '<...>\Blocks\WindowsNoEditor'
BLOCKS_WIN_PATH = os.environ.get('CITYNAV_BLOCKS_PATH', '')
RPC_PORT = 45481
TCP_PORT = 4581
SAM_PORT, CLIP_PORT = 8100, 8101
AGENT_PORT = 8971
PX4_INSTANCE, PX4_SYSTEM_ID = 70, 71
TASK_NAME = 'CityNavBlocks'
# z-oscillation attribution (log/citynav/z-limcycle): allow overriding the
# Blocks render-load arguments per experiment without editing the file.
BLOCKS_ARGS = os.environ.get(
    'CITYNAV_BLOCKS_ARGS',
    '-NoSplash -windowed -ResX=1024 -ResY=1024 -nosound '
    '"-ExecCmds=r.Streaming.PoolSize 128"')
ROS_WS = '/repo/log/citynav/pc-e2e/ros_ws'
WORKSTATION_WS = '/repo/log/citynav/pc-e2e/workstation/ws'
CLOUD_ENV = str(ROOT / 'log' / 'citynav' / 'pc-e2e' / 'cloud-env')
INFERENCE_CONFIG = str(ROOT / 'config' / 'inference.cloud.local.yaml')


def run(args, **kwargs):
    return subprocess.run(list(map(str, args)), check=True, **kwargs)


def capture(args):
    return subprocess.check_output(list(map(str, args)), text=True,
                                   errors='replace', stderr=subprocess.STDOUT)


def powershell(script):
    return capture(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass',
                    '-Command', script])


def domain(output):
    path = output / 'domain.json'
    return json.loads(path.read_text())['domain'] if path.exists() else 190


def prefix(output):
    return 'citynav-' + hashlib.sha256(str(output).encode()).hexdigest()[:8] + '-'


def common(output):
    # Same DDS posture as the validated SPF runs: host networking inside the
    # Docker Desktop VM keeps every role in one network namespace (cross-role
    # discovery+data), ROS_LOCALHOST_ONLY=0 + the shared UDPv4 profile.
    return ['--init', '--network', 'host',
            '-e', 'ROS_DOMAIN_ID=' + str(domain(output)),
            '-e', 'ROS_LOCALHOST_ONLY=0',
            '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/repo/deployment/airsim/config/dds/fastdds_udp.xml',
            '-e', 'PYTHONDONTWRITEBYTECODE=1',
            '-v', str(ROOT) + ':/repo:ro', '-v', str(output) + ':/evidence']


def launch(output, role, image, args, env=(), mounts=(), entrypoint=None):
    command = ['docker', 'run', '-d', '--name', prefix(output) + role,
               '--label', 'aerohub.citynav.output=' + str(output),
               *common(output), *env, *mounts]
    if entrypoint:
        command += ['--entrypoint', entrypoint]
    command += [image, *args]
    run(command)


def check_owner(output, role):
    name = prefix(output) + role
    label = capture(['docker', 'inspect', '--format',
                     '{{index .Config.Labels "aerohub.citynav.output"}}', name]).strip()
    if label != str(output):
        raise RuntimeError('resource owner mismatch')
    return name


def restart_blocks(output, observe=False):
    """Blocks must be fresh so PX4 is the first TCP client (single-slot server)."""
    if not BLOCKS_WIN_PATH:
        raise RuntimeError(
            'CITYNAV_BLOCKS_PATH is not set; point it at your local Blocks '
            'install directory (the one containing Blocks.exe) before '
            'running start/mission')
    settings = json.loads((APP / 'settings.json').read_text())
    if observe:
        # AGENTS multi-deployment rule: centroid-tracking top-down view through
        # the shared deployment/swarm infrastructure.
        sys.path.insert(0, str(ROOT / 'deployment' / 'swarm'))
        from view import apply_overhead_view
        settings = apply_overhead_view(settings)
    (output / 'settings.json').write_text(json.dumps(settings, indent=2) + '\n')
    win_settings = str((output / 'settings.json').resolve())
    powershell(
        "$s = Get-Content -Raw '" + win_settings + "'; "
        "if (-not $s) { throw 'settings.json read failed' } "
        "$dir = Join-Path $env:USERPROFILE 'Documents\\AirSim'; "
        "New-Item -ItemType Directory -Force -Path $dir | Out-Null; "
        "if (Test-Path (Join-Path $dir 'settings.json')) { Copy-Item (Join-Path $dir 'settings.json') (Join-Path $dir ('settings.backup.' + (Get-Date -Format 'yyyyMMddHHmmss') + '.json')) }; "
        "Set-Content -Path (Join-Path $dir 'settings.json') -Value $s -Encoding utf8; "
        "taskkill /F /IM Blocks.exe 2>$null; Start-Sleep 4")
    # Launch Blocks through a Windows scheduled task, NOT a direct
    # Start-Process: a GUI process started as a child of the agent's shell is
    # reaped when the command returns (SPF finding, same job-object cause).
    # The window resolution MUST be >= any camera capture resolution used
    # through the scene render target (SPF: mismatch pegs the CPU and
    # destabilises the lockstep mavlink link). 1280x720 covers 256x256.
    # A running scheduled task CANNOT be deleted (schtasks returns an error,
    # previously swallowed by 2>$null), so a stale world could survive `stop`.
    # Kill the image, verify zero remain, then delete + recreate the task.
    # NOTE: this Blocks build is a launcher stub (WindowsNoEditor\Blocks.exe)
    # that spawns the real UE4 process (...\Blocks\Binaries\Win64\Blocks.exe);
    # a healthy single session therefore shows TWO Blocks.exe PIDs. Readiness
    # is counted on the child binary only (run d: counting all instances as
    # duplicates was wrong).
    powershell(
        "taskkill /F /IM Blocks.exe 2>$null; Start-Sleep 4; "
        "$left = @(Get-Process Blocks -ErrorAction SilentlyContinue).Count; "
        "if ($left -ne 0) { throw \"$left Blocks.exe instance(s) survived taskkill\" } "
        "schtasks /delete /tn " + TASK_NAME + " /f 2>$null | Out-Null; "
        # Host VRAM/render-target mitigation. Blocks.log (run j) proved the
        # crash is NOT VRAM OOM but UE4 BitArray.h assertion from "Reallocating
        # scene render targets" thrash: the overhead observer's external
        # camera captures at 1024x1024 (deployment/swarm/view.py), so the
        # window must be >= 1024 in BOTH dimensions or UE4 reallocates the
        # scene RT every frame and dies (runs c/f/g/h crashed at 1280x720,
        # run i/j at 1024x576/800x450). Texture pool capped to leave room for
        # the resident inference services. CITYNAV_BLOCKS_ARGS overrides the
        # render-load arguments for the z-oscillation attribution experiments
        # (inner double quotes are escaped for the PowerShell single-quoted
        # string, matching the historical working form).
        "$tr = '" + BLOCKS_WIN_PATH + "\\\\Blocks.exe "
        + BLOCKS_ARGS.replace('"', '\\"') + "'; "
        "schtasks /create /tn " + TASK_NAME + " /tr $tr /sc once /st 00:00 /f | Out-Null; "
        "schtasks /run /tn " + TASK_NAME)
    deadline = time.time() + 120
    while True:
        procs = powershell(
            "@(Get-CimInstance Win32_Process -Filter \"Name='Blocks.exe'\" | "
            "Where-Object { $_.CommandLine -match 'Binaries' } | Measure-Object).Count").strip()
        if procs == '1':
            break
        if time.time() > deadline:
            raise RuntimeError('UE4 Blocks child process count %s (expected exactly 1)' % procs)
        time.sleep(2)
    # Process alive != world ready: wait for the PX4 TCP slot to listen.
    deadline = time.time() + 120
    while True:
        listening = powershell(
            "netstat -ano | Select-String ':%d.*LISTENING'" % TCP_PORT)
        if listening.strip():
            break
        if time.time() > deadline:
            raise RuntimeError('Blocks TCP %d never listened' % TCP_PORT)
        time.sleep(2)


def windows_ipv4_candidates():
    out = powershell(
        "(Get-NetIPAddress -AddressFamily IPv4 | "
        "Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.*' } "
        "| Select-Object -ExpandProperty IPAddress) -join ','")
    return [ip.strip() for ip in out.strip().split(',') if ip.strip()]


def probe_host(output):
    """Pick the address containers use for Blocks RPC + inference services.

    IPv4 literals first: Blocks/PX4 listen and connect over IPv4 only (run a:
    host.docker.internal resolves to an IPv6 ULA that the Python RPC client
    tolerates via fallback but PX4's C++ netlib does not, leaving
    simulator_mavlink stuck in "Waiting for simulator"). NAT/vEthernet
    adapters (172.*) are preferred over LAN-facing addresses for isolation.
    """
    ips = windows_ipv4_candidates()
    ips.sort(key=lambda ip: 0 if ip.startswith('172.') else 1)
    candidates = ips + ['host.docker.internal']
    command = ['docker', 'run', '--rm', '--network', 'host',
               '-v', str(ROOT) + ':/repo:ro', '-v', str(output) + ':/evidence',
               '--entrypoint', 'python3', GROUND,
               '/repo/deployment/airsim/demos/citynav_blocks/host_probe.py',
               str(RPC_PORT), str(SAM_PORT), str(CLIP_PORT), *candidates]
    out = capture(command)
    (output / 'host-probe.json').write_text(out)
    chosen = json.loads(out.strip().splitlines()[-1])
    if not chosen.get('chosen'):
        raise RuntimeError('no host address reached Blocks RPC + inference; '
                           'see host-probe.json')
    return chosen['chosen'], chosen


def pick_domain(output):
    # Only RUNNING containers can occupy a DDS domain; exited/stopped ones
    # hold no sockets. The earlier union of every historical domain.json
    # exhausted the 180-232 window after ~20 runs (tool11 start, 2026-11-18)
    # - recorded files are evidence, not reservations.
    used = set()
    identifiers = capture(['docker', 'ps', '-q']).split()
    if identifiers:
        for container in json.loads(capture(['docker', 'inspect', *identifiers])):
            for entry in container['Config']['Env']:
                if entry.startswith('ROS_DOMAIN_ID='):
                    used.add(int(entry.split('=', 1)[1]))
    available = [v for v in range(180, 233) if v not in used]
    # 180-199 (the SPF-era window); Fast DDS accepts domain ids up to 232
    # (RTPS port arithmetic stays in range).
    if not available:
        raise RuntimeError('no unused DDS domain')
    (output / 'domain.json').write_text(json.dumps({'domain': available[0]}) + '\n')


def generate_planner_params(output, host):
    (output / 'planner.yaml').write_text((APP / 'planner.yaml').read_text())
    (output / 'inference.yaml').write_text(
        (APP / 'inference.yaml').read_text().replace('__HOST__', host))


def launch_compute(output):
    launch(output, 'compute', ONBOARD,
           ['-lc', 'exec bash /repo/deployment/airsim/demos/citynav_blocks/citynav_compute.sh'],
           env=['-e', 'CITYNAV_FLEET_FILE=/evidence/fleet.yaml',
                '-e', 'CITYNAV_PLANNER_PARAMS=/evidence/planner.yaml'],
           entrypoint='bash')


def launch_inference(output):
    import yaml

    cloud = yaml.safe_load(Path(INFERENCE_CONFIG).read_text())['models']['vlm']['cloud']
    environment = []
    for field in ('base_url_env', 'model_env', 'api_key_env'):
        key = cloud.get(field)
        if key and key in os.environ:
            environment += ['-e', key]
    launch(output, 'inference', ONBOARD, ['-lc',
           'source /opt/ros/humble/setup.bash && '
           'source ' + ROS_WS + '/install/setup.bash && '
           'exec python3 /repo/deployment/inference/vision_language_run.py'],
           env=[*environment, '-e', 'INFERENCE_IMAGE_TEXT=1',
                '-e', 'INFERENCE_MODEL_CONFIG=/run/inference/models.yaml',
                '-e', 'INFERENCE_ROS_PARAMETERS=/evidence/inference.yaml',
                '-e', 'VEHICLE_NAMESPACE=drone0'],
           mounts=['-v', CLOUD_ENV + ':/cloud-env:ro',
                   '-v', INFERENCE_CONFIG + ':/run/inference/models.yaml:ro'],
           entrypoint='bash')


def start(output, observe=False):
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'images.json').exists():
        raise RuntimeError('use a fresh output directory')
    pick_domain(output)
    for image in [FLIGHT, ONBOARD, GROUND]:
        run(['docker', 'image', 'inspect', image], stdout=subprocess.DEVNULL)
    (output / 'images.json').write_text(json.dumps(
        {i: capture(['docker', 'image', 'inspect', i, '--format', '{{.Id}}']).strip()
         for i in [FLIGHT, ONBOARD, GROUND]}, indent=2) + '\n')
    (output / 'fleet.yaml').write_text(
        'vehicles:\n  drone0:\n    fmu_prefix: /drone0\n    target_system_id: %d\n' % PX4_SYSTEM_ID)
    source_manifest = {'files': {}, 'citynav_upstream': capture(
        ['git', '-C', ROOT / 'src/tool/navigation/planner/citynav/upstream',
         'rev-parse', 'HEAD']).strip()}
    for folder in [APP, ROOT / 'src/tool/navigation/planner/citynav/ros2_adapter',
                   ROOT / 'src/tool/navigation/interfaces', ROOT / 'src/behavior/follow_path',
                   ROOT / 'src/basic_function/inference/interfaces',
                   ROOT / 'src/basic_function/inference/ros2_adapter/uav_inference',
                   ROOT / 'deployment/inference']:
        for path in folder.rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts \
                    and '.pytest_cache' not in path.parts:
                source_manifest['files'][str(path.relative_to(ROOT))] = \
                    hashlib.sha256(path.read_bytes()).hexdigest()
    (output / 'source-manifest.json').write_text(json.dumps(source_manifest, indent=2) + '\n')

    restart_blocks(output, observe)
    host, probe = probe_host(output)
    (output / 'topology.json').write_text(json.dumps(
        {'host_address': host, 'rpc_port': RPC_PORT, 'tcp_port': TCP_PORT,
         'sam_port': SAM_PORT, 'clip_port': CLIP_PORT,
         'blocks_path': BLOCKS_WIN_PATH, 'overhead_view': bool(observe),
         'host_probe': probe['report']}, indent=2) + '\n')
    generate_planner_params(output, host)

    launch(output, 'vehicle', FLIGHT, ['vehicle'],
           env=['-e', 'VEHICLE_NAME=PX4', '-e', 'VEHICLE_NAMESPACE=drone0',
                '-e', 'PX4_INSTANCE=%d' % PX4_INSTANCE, '-e', 'PX4_SYSTEM_ID=%d' % PX4_SYSTEM_ID,
                '-e', 'XRCE_AGENT_PORT=%d' % AGENT_PORT,
                '-e', 'AIRSIM_HOST=' + host, '-e', 'AIRSIM_RPC_PORT=%d' % RPC_PORT,
                '-e', 'FLIGHT_BOARD_SENSORS=0', '-e', 'FLIGHT_BOARD_CAMERAS=0'],
           mounts=['-v', str(output / 'settings.json') + ':/config/settings.json:ro'])
    launch(output, 'clock', FLIGHT, ['clock'],
           env=['-e', 'VEHICLE_NAME=PX4', '-e', 'AIRSIM_HOST=' + host,
                '-e', 'AIRSIM_RPC_PORT=%d' % RPC_PORT],
           mounts=['-v', str(output / 'settings.json') + ':/config/settings.json:ro'])
    deadline = time.monotonic() + 120
    while 'Ready for takeoff!' not in capture(['docker', 'logs', prefix(output) + 'vehicle']):
        if time.monotonic() > deadline:
            raise RuntimeError('PX4 preflight timeout; preserve resources for diagnosis')
        time.sleep(.5)
    launch(output, 'sensor', FLIGHT, ['-lc',
           'source /opt/ros/humble/setup.bash && source /opt/platform_sensors/install/setup.bash && '
           'exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args '
           '-p settings_file:=/config/settings.json -p rpc_host:=' + host +
           ' -p rpc_port:=%d ' % RPC_PORT +
           '-p vehicle_name:=PX4 -p vehicle_namespace:=drone0 '
           '-p camera_rate_hz:=5.0 -p use_sim_time:=true'],
           mounts=['-v', str(output / 'settings.json') + ':/config/settings.json:ro'],
           entrypoint='bash')
    launch_compute(output)
    launch_inference(output)


def health(output):
    topology = json.loads((output / 'topology.json').read_text())
    host = topology['host_address']
    stamp = time.strftime('%Y%m%dT%H%M%S')
    report = 'health-' + stamp + '.json'
    run(['docker', 'run', '--rm', *common(output),
         '--entrypoint', 'bash', GROUND, '-lc',
         'source /opt/ros/humble/setup.bash && '
         'source ' + ROS_WS + '/install/setup.bash && '
         'source ' + WORKSTATION_WS + '/install/setup.bash && '
         'exec python3 /repo/deployment/airsim/demos/citynav_blocks/health.py '
         '--output /evidence/' + report +
         ' --grounding-health http://' + host + ':%d/health/ready' % SAM_PORT +
         ' --clip-health http://' + host + ':%d/health/ready' % CLIP_PORT])
    print((output / report).read_text())


def observer_command(output, case, host, once):
    cmd = ['docker', 'run']
    if once:
        cmd += ['--rm']
    else:
        cmd += ['-d', '--name', prefix(output) + 'observer',
                '--label', 'aerohub.citynav.output=' + str(output)]
    cmd += common(output) + [
        '-e', 'PYTHONPATH=/repo/simulator/airsim/sensors:/repo/deployment/robopi/vendor',
        '--entrypoint', 'bash', GROUND, '-lc',
        ('exec ' if not once else '') +
        'python3 /repo/deployment/airsim/demos/citynav_blocks/observer.py '
        '--settings /evidence/settings.json --host ' + host +
        ' --evidence /evidence/' + case + (' --once' if once else '')]
    return cmd


def mission(output, args):
    case_path = output / args.case
    if case_path.exists():
        raise RuntimeError('use a fresh case name; evidence must not be overwritten')
    topology = json.loads((output / 'topology.json').read_text())
    host = topology['host_address']
    observe = bool(topology.get('overhead_view'))
    command = ['docker', 'run', '--rm', *common(output),
               '-e', 'PYTHONPATH=/repo/simulator/airsim/sensors:/repo/deployment/robopi/vendor',
               '--entrypoint', 'bash', GROUND, '-lc',
               'source /opt/ros/humble/setup.bash && '
               'source ' + ROS_WS + '/install/setup.bash && '
               'source ' + WORKSTATION_WS + '/install/setup.bash && '
               'exec python3 /repo/deployment/airsim/demos/citynav_blocks/mission.py '
               '--output /evidence/' + args.case +
               ' --instruction "' + args.instruction + '"'
               ' --max-steps ' + str(args.max_steps) +
               ' --timeout-budget ' + str(args.timeout_budget) +
               ' --use-memory ' + str(int(args.use_memory)) +
               ' --cancel-after ' + str(args.cancel_after) +
               (' --expect-fault' if args.expect_fault else '') +
               (' --expect-stuck' if args.expect_stuck else '') +
               ' --timeout ' + str(args.timeout) +
               ' --rpc-host ' + host + ' --rpc-port %d' % RPC_PORT +
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
            health_out = capture(observer_command(output, args.case, host, True))
            (output / (args.case + '.observer-health.json')).write_text(health_out)
            run(observer_command(output, args.case, host, False))
            observer = prefix(output) + 'observer'
        run(command)
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
    # task never reached flight (readiness/teleport/takeoff failures leave it
    # on the ground). Anything from navigation onward must confirm disarm.
    error = str(latest.get('error', ''))
    safe_on_ground = any('at ' + stage + ':' in error
                         for stage in ('readiness', 'teleport', 'takeoff'))
    # If the simulator itself died mid-task (runs c/f: Blocks BEX64 crash
    # during navigation), there is no drone left to land; the process check
    # is authoritative, the error-text match is only a fallback.
    blocks_alive = powershell(
        "(Get-Process Blocks -ErrorAction SilentlyContinue | Measure-Object).Count").strip()
    simulator_gone = blocks_alive == '0' or (
        'AirSim closed the RPC connection' in error or 'timed out' in error
        or 'Connection refused' in error)
    if results and not (latest.get('disarmed') or safe_on_ground
                        or simulator_gone
                        # explicit external recovery confirmation artifact
                        # (e.g. land_once.py output after a mission that
                        # timed out reading the Land result)
                        or (output / 'disarmed-confirmed.json').exists()):
        raise RuntimeError('latest task must confirm disarmed before cleanup')
    observer = prefix(output) + 'observer'
    if capture(['docker', 'ps', '-aq', '--filter', 'name=' + observer]).strip():
        run(['docker', 'rm', '-f', observer])
    for role in ['inference', 'compute', 'sensor', 'clock', 'vehicle']:
        name = prefix(output) + role
        if not capture(['docker', 'ps', '-aq', '--filter', 'name=' + name]).strip():
            continue  # start died before this role was created
        label = capture(['docker', 'inspect', '--format',
                         '{{index .Config.Labels "aerohub.citynav.output"}}', name]).strip()
        if label != str(output):
            raise RuntimeError('resource owner mismatch')
        (output / (role + '.txt')).write_text(capture(['docker', 'logs', name]))
        run(['docker', 'stop', '--timeout', '15', name])
        run(['docker', 'rm', name])
    powershell('taskkill /F /IM Blocks.exe 2>$null; '
               'Start-Sleep 3; '
               '$left = @(Get-Process Blocks -ErrorAction SilentlyContinue).Count; '
               'if ($left -ne 0) { taskkill /F /IM Blocks.exe /T 2>$null; Start-Sleep 3 }; '
               'schtasks /delete /tn ' + TASK_NAME + ' /f 2>$null | Out-Null')
    left = powershell(
        "(Get-Process Blocks -ErrorAction SilentlyContinue | Measure-Object).Count").strip()
    if left != '0':
        raise RuntimeError('%s Blocks.exe instance(s) survived cleanup' % left)


def restart_compute(output):
    """Re-create compute + inference after an overlay rebuild (both source it).
    Tolerates roles already removed (e.g. a crashed inference pruned by hand)."""
    for role in ('inference', 'compute'):
        name = prefix(output) + role
        if capture(['docker', 'ps', '-aq', '--filter', 'name=' + name]).strip():
            run(['docker', 'rm', '-f', check_owner(output, role)])
    launch_compute(output)
    launch_inference(output)


def prepare(output):
    """Build the citynav packages + shared interfaces + follow_path into the
    ros_ws overlay, and the canonical as2_msgs subset for the workstation.

    The repository is bind-mounted at /repo; all in-container paths use the
    mount point (the Windows host path contains non-ASCII characters)."""
    work = ROS_WS
    (ROOT / 'log/citynav/pc-e2e/ros_ws').mkdir(parents=True, exist_ok=True)
    run(['docker', 'run', '--rm', '-v', str(ROOT) + ':/repo', '-w', work,
         '--entrypoint', 'bash', ONBOARD, '-lc',
         'source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && '
         'cd ' + work + ' && colcon build --base-paths '
         '/repo/src/tool/navigation/interfaces '
         '/repo/src/tool/navigation/planner/citynav/ros2_adapter '
         '/repo/src/basic_function/inference/interfaces '
         '/repo/src/basic_function/inference/ros2_adapter '
         '/repo/src/behavior/follow_path '
         '--packages-select uav_planner_interfaces uav_citynav_planner '
         'uav_inference_interfaces uav_inference '
         'uav_follow_path_execution '
         '--event-handlers console_direct+'])
    prepare_cloud_worker()
    prepare_ground()


def prepare_cloud_worker():
    """uv-locked isolated worker env for the shared vision-language model
    (serves /infer and /infer_frames), built inside the target image so the
    Python ABI matches the runtime container (SPF same method)."""
    import shutil
    uv = shutil.which('uv')
    if not uv:
        raise RuntimeError('Install uv to prepare the isolated Cloud worker environment')
    (ROOT / 'log/citynav/pc-e2e').mkdir(parents=True, exist_ok=True)
    run(['docker', 'run', '--rm', '-v', str(ROOT) + ':/repo:ro',
         '-v', CLOUD_ENV + ':/cloud-env',
         '-v', uv + ':/usr/local/bin/uv:ro', '--entrypoint', 'bash', ONBOARD, '-lc',
         'mkdir -p /tmp/cloud-build/models/vision_language /tmp/cloud-build/runtime && '
         'cp -a /repo/src/basic_function/inference/runtime/. /tmp/cloud-build/runtime/ && '
         'cp /repo/src/basic_function/inference/models/vision_language/pyproject.toml '
         '/repo/src/basic_function/inference/models/vision_language/uv.lock /tmp/cloud-build/models/vision_language/ && '
         'UV_PROJECT_ENVIRONMENT=/cloud-env uv sync --locked --reinstall-package aerohub-inference-runtime '
         '--python /usr/bin/python3 --project /tmp/cloud-build/models/vision_language'])


def prepare_ground():
    """Build the canonical as2_msgs subset that mission.py consumes (SPF pattern).

    The workstation image has no Aerostack2; only the interfaces the task
    actually uses are generated in its own environment, pinned by image IDs
    and file hashes."""
    ws = ROOT / 'log/citynav/pc-e2e/workstation/ws'
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
         '-w', '/repo/log/citynav/pc-e2e/workstation/ws', '--entrypoint', 'bash', GROUND, '-lc',
         'source /opt/ros/humble/setup.bash && colcon build --event-handlers console_direct+'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'start', 'health', 'mission',
                                           'stop', 'restart-compute'])
    parser.add_argument('--output', type=Path, default=None)
    parser.add_argument('--case', default='formal1')
    parser.add_argument('--instruction',
                        default='fly toward the orange ball and stop near it')
    parser.add_argument('--max-steps', type=int, default=60)
    parser.add_argument('--timeout-budget', type=float, default=400.0,
                        help='mission action only: to_semantic_goal budget '
                             '(inference + motion), enforced by the Tool')
    parser.add_argument('--use-memory', type=int, choices=[0, 1], default=1)
    parser.add_argument('--cancel-after', type=float, default=0.0,
                        help='mission action only: inject citynav/cancel N s '
                             'after the goal (0 disables; formal1 keeps it off)')
    parser.add_argument('--expect-fault', action='store_true',
                        help='mission action only: M6 fault1 gates (the LLM '
                             'path was made persistently unreachable via a '
                             'dead models.vlm.cloud.base_url in the local Inference config '
                             '+ restart-compute before running the case)')
    parser.add_argument('--expect-stuck', action='store_true',
                        help='mission action only: M6 stuck1 gates (grounding '
                             'repointed to the always-empty host stub in '
                             '<output>/inference.yaml (grounded_sam_endpoint) '
                             '+ restart-compute; run '
                             'with a >=4-landmark instruction)')
    parser.add_argument('--timeout', type=float, default=900.)
    parser.add_argument('--target', type=float, nargs=3, default=[34.25, 33.10, 3.68],
                        help='ground-truth OrangeBall in odom ENU (NED z=-3.68)')
    parser.add_argument('--spawn', type=float, nargs=3, default=[10.0, 10.0, 45.0],
                        help='teleport spawn x y yaw_deg at ground level '
                             '(default 10,10: the task-spec -10,-10 puts the '
                             'OrangeBall at ~62 m, beyond the contract 50 m '
                             'depth gate, so mask pixels backproject to walls '
                             'instead of the ball - run-20261118f collision '
                             'FAIL; see README deviation)')
    parser.add_argument('--observe', action='store_true',
                        help='start action only: enable the centroid-tracking '
                             'overhead view (AGENTS M7); missions then run an '
                             'observer container and record overhead evidence')
    args = parser.parse_args()
    if args.output is None:
        raise ValueError('--output required (inside log/citynav/pc-e2e)')
    output = args.output.resolve()
    if not str(output).startswith(str(ROOT / 'log')):
        raise ValueError('output must be inside repository log/')
    if args.action == 'prepare':
        prepare(output)
    elif args.action == 'start':
        start(output, args.observe)
    elif args.action == 'health':
        health(output)
    elif args.action == 'mission':
        mission(output, args)
    elif args.action == 'stop':
        stop(output)
    elif args.action == 'restart-compute':
        restart_compute(output)


if __name__ == '__main__':
    main()
