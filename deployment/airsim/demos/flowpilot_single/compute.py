"""FlowPilot compute supervisor: AS2 stack, inference adapter, planner.

Mirrors yopo_single compute.py. If the planner or adapter exits, AS2 is kept
running for bounded trajectory expiry and native Land recovery.
"""
import os
import signal
import subprocess
import time

ROOT = '/repo'


def main():
    # This deployment tune applies only to the FlowPilot AirSim/PX4 SITL demo.
    os.environ.setdefault('AS2_CONTROLLER_CONFIG_FILE',
        ROOT + '/deployment/airsim/demos/flowpilot_single/pid_speed_controller.yaml')
    endpoint = os.environ.get('FLOWPILOT_ENDPOINT', 'http://127.0.0.1:18082')
    adapter_timeout = os.environ.get('FLOWPILOT_ADAPTER_TIMEOUT_S', '1.0')
    # Depth frames arrive in bursts (~1 Hz) with gaps of 0.55-1.0 s, so the
    # adapter default of 0.3 s rejects most observations.  The verified
    # configuration uses 2.0 s; keep it configurable through the environment.
    adapter_max_input_age = os.environ.get('FLOWPILOT_MAX_INPUT_AGE_S', '2.0')
    params = os.environ.get(
        'FLOWPILOT_PLANNER_PARAMS',
        ROOT + '/deployment/airsim/demos/flowpilot_single/planner.yaml')
    commands = [
        ['python3', ROOT + '/deployment/airsim/as2/run.py'],
        ['python3', '-m', 'uav_inference.flowpilot_node', '--ros-args',
         '-r', '__ns:=/drone0', '-p', 'use_sim_time:=true',
         '-p', 'endpoint:=' + endpoint, '-p', 'timeout_s:=' + adapter_timeout,
         '-p', 'max_input_age_s:=' + adapter_max_input_age],
        ['ros2', 'run', 'uav_flowpilot_planner', 'flowpilot_planner', '--ros-args',
         '-r', '__ns:=/drone0', '--params-file', params,
         '-r', 'flowpilot/depth:=sensor_measurements/flowpilot_front/depth_planar/image',
         '-r', 'flowpilot/camera_info:=sensor_measurements/flowpilot_front/depth_planar/camera_info'],
    ]
    children = []
    stopping = False
    reported = set()

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        for command in commands:
            print('Starting:', ' '.join(command), flush=True)
            children.append(subprocess.Popen(command, start_new_session=True))
        while not stopping:
            if children[0].poll() is not None:
                raise RuntimeError('AS2 execution process exited')
            for index, child in enumerate(children[1:], 1):
                if child.poll() is not None and index not in reported:
                    print('Planning process exited; retain AS2 for bounded trajectory '
                          'expiry and native Land recovery:', index, child.returncode, flush=True)
                    reported.add(index)
            time.sleep(.1)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=3)


if __name__ == '__main__':
    main()
