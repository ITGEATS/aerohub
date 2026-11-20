"""Real mapping/planning/RPC, synthetic RGB-D/state/clock; no flight clients.

The camera fixture is the locked Fork's calibration, used only as synthetic
test geometry. Test wait budgets reuse the existing native smoke's 6/10 s.
"""
import base64
import copy
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rosgraph_msgs.msg import Clock
from quadrotor_interfaces.msg import PolyTraj
from std_msgs.msg import Empty
from uav_elastic_tracker_adapter.transport import rpc

ROOT = Path('/workspace')
C = ROOT / 'src/tool/navigation/planner/elastic_tracker'


def main():
    rclpy.init()
    node = Node('elastic_native_replay')
    clock = node.create_publisher(Clock, '/clock', 1)
    values = {'ns': 100_000_000_000, 'paused': False}
    received = []
    heartbeats = []
    node.create_subscription(PolyTraj, '/elastic/trajectory', received.append, 10)
    node.create_subscription(Empty, '/elastic/heartbeat', heartbeats.append, 10)

    def tick():
        if not values['paused']:
            values['ns'] += 20_000_000  # synthetic clock step, not a product rate change
        message = Clock()
        message.clock.sec, message.clock.nanosec = divmod(values['ns'], 1_000_000_000)
        clock.publish(message)
    node.create_timer(.02, tick)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    output = Path(os.environ.get('ELASTIC_TEST_OUTPUT', '/build/native-replay-' + str(time.time_ns())))
    output.mkdir(parents=True, exist_ok=False)
    print('Replay evidence:', output, flush=True)
    log = (output / 'process.log').open('w')
    # Drive the compatibility endpoint directly; no execution Adapter owns this fixture.
    process = subprocess.Popen(['ros2', 'launch', 'uav_elastic_tracker_adapter', 'native.launch.py',
        'camera_config:=' + str(C / 'config/calibration-example.yaml'),
        'fresh_native_output:=true'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    call = lambda payload: rpc('127.0.0.1', 28500, payload)

    def wait(predicate, budget=6.):
        deadline = time.monotonic() + budget
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError('Native exited early; see native-replay-process.log')
            try:
                result = predicate()
                if result:
                    return result
            except (OSError, RuntimeError):
                pass
            threading.Event().wait(.02)
        raise AssertionError('Replay wait expired; see native-replay-process.log')

    def inputs():
        stamp = list(divmod(values['ns'], 1_000_000_000))
        call({'kind': 'clock', 'stamp': stamp})
        message = {'header': {'stamp': stamp, 'frame_id': 'drone0/odom'},
                   'child_frame_id': 'drone0/base_link', 'p': [0., 0., 1.5],
                   'q': [0., 0., 0., 1.], 'v': [0., 0., 0.], 'w': [0., 0., 0.],
                   'pose_covariance': [0.] * 36, 'twist_covariance': [0.] * 36}
        call({'kind': 'odom', 'message': message})
        target = copy.deepcopy(message)
        target['p'] = [3., 0., .5]
        target['v'] = [.1, 0., 0.]
        call({'kind': 'target', 'message': target})
        depth = {'header': {'stamp': stamp, 'frame_id': 'synthetic_camera_optical'},
                 'height': 480, 'width': 640, 'encoding': '16UC1', 'is_bigendian': 0,
                 'step': 1280, 'data': base64.b64encode(struct.pack('<H', 6000) * (640 * 480)).decode()}
        call({'kind': 'depth', 'message': depth})
        return call({'kind': 'health'})

    try:
        # Deployment discovery budget from deployment/onboard_computer/check_nodes.py:24.
        # The 6 s business waits start after imports and node startup, as in the
        # original fixture; Windows bind-mount Python import time is not RPC age.
        wait(lambda: call({'kind': 'health'})['source_connected'], budget=25.)
        wait(lambda: inputs()['map_received'])
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(call, {'kind': 'latest', 'task_id': 'synthetic-replay'})
            wait(lambda: (inputs(), future.done())[1])
            response = future.result()
        native = response['trajectory']
        assert native and native['start_time'][0] >= 100 and native['generated_at'][0] >= 100
        assert received, 'No actual DDS trajectory observed'
        health = call({'kind': 'health'})
        assert set(('clock', 'odom', 'target', 'depth', 'latest', 'health')) <= set(health['inputs'])
        assert health['source_connected'] and health['map_received']
        native_clock_consumers = {info.node_name for info in
                                  node.get_subscriptions_info_by_topic('/elastic/native_clock')}
        assert {'mapping', 'planning'} <= native_clock_consumers
        assert node.count_publishers('/elastic/native_clock') == 1
        # Only this synthetic fixture owns /clock; endpoint must not echo RPC clocks.
        assert node.count_publishers('/clock') == 1
        print('PASS native depth/odom/target -> map -> planning -> RPC; all six kinds; sole clock', flush=True)

        values['paused'] = True
        frozen = values['ns']
        wait(lambda: call({'kind': 'health'})['clock_ns'] == frozen and heartbeats)
        # Let in-flight callbacks/messages settle before observing the pause.
        threading.Event().wait(.2)
        trajectory_count, heartbeat_count = len(received), len(heartbeats)
        for _ in range(5):
            inputs()
            threading.Event().wait(.05)
        assert call({'kind': 'health'})['clock_ns'] == frozen
        assert len(heartbeats) == heartbeat_count, 'Planner callback fired during frozen ROS time'
        assert len(received) == trajectory_count, 'Planner emitted a trajectory during frozen ROS time'
        print('PASS paused clock: no planning callbacks or trajectories', flush=True)

        values['paused'] = False
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(call, {'kind': 'latest', 'task_id': 'synthetic-replay'})
            wait(lambda: (inputs(), future.done())[1])
            resumed = future.result()['trajectory']
        assert resumed['generated_at'] > list(divmod(frozen, 1_000_000_000))
        wait(lambda: len(heartbeats) > heartbeat_count and len(received) > trajectory_count)
        print('PASS clock resumes: planner publishes with an advanced ROS timestamp', flush=True)
        values['paused'] = True
        values['ns'] = 50_000_000_000
        process.wait(timeout=10.)
        wait_end = time.monotonic() + 6.
        while any(name in node.get_node_names() for name in ('mapping', 'planning', 'elastic_tracker_native_rpc',
                                                             'elastic_tracker_inputs', 'elastic_native_adapter')):
            if time.monotonic() >= wait_end:
                raise AssertionError('Native nodes survived rewind shutdown')
            threading.Event().wait(.02)
        print('PASS rewind closes RPC and exits mapping/planning; explicit restart required', flush=True)
        processes = subprocess.check_output(['ps', '-eo', 'comm,args'], text=True)
        assert not any(line.split()[0] in ('roscore', 'rosmaster') for line in processes.splitlines() if line.split())
        print('PASS no ROS 1 master/process or flight API; synthetic input only', flush=True)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=6.)
                print('CLEANUP: native launch required SIGKILL', flush=True)
        log.close()
        executor.shutdown()
        thread.join(timeout=6.)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
