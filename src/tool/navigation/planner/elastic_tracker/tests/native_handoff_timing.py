"""Isolated native and timed execution with synthetic inputs/controller.

Trace identifies queueing and clock progress without changing source epochs.
Discovery/business budgets reuse the existing 25s/6s smoke budgets.
"""
import argparse
from array import array
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger
import yaml
from native_ros_smoke import Fixture, old, ROOT
from uav_planner_interfaces.action import FollowTimedPath


class Inputs(Fixture):
    def __init__(self):
        super().__init__()
        self.destroy_publisher(self.body)
        self.destroy_publisher(self.target)
        # Native input_node is the sole accepted state/target publisher.
        self.body = self.target = SimpleNamespace(publish=lambda _: None)
        self.raw = self.create_publisher(Odometry, '/drone0/sensor_measurements/odom', 10)
        self.raw_target = self.create_publisher(Odometry, '/elastic_tracker/target_odom', 10)
        self.depth_pub = self.create_publisher(Image, '/synthetic/depth', qos_profile_sensor_data)
        camera = yaml.safe_load((ROOT/'src/tool/navigation/planner/elastic_tracker/config/calibration-example.yaml').read_text())
        self.depth_timer = self.create_timer(1./camera['camera_rate'], self.depth_frame)
    def pose(self):
        super().pose()
        if not hasattr(self, 'raw'):
            return
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'drone0/odom'
        message.child_frame_id = 'drone0/base_link'
        message.pose.pose.position.x = self.x
        message.pose.pose.orientation.w = 1.
        self.raw.publish(message)
        message = copy.deepcopy(message)
        message.child_frame_id = 'drone0/odom'
        # Explicit mathematical fixture: native +1m offset yields z=0.
        message.pose.pose.position.x, message.pose.pose.position.z = 10., -1.
        self.raw_target.publish(message)

    def depth_frame(self):
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'synthetic_camera_optical'
        message.width, message.height = 640, 480
        message.encoding, message.step = '32FC1', 640*4
        # Unchanged native zero/no-return conversion; not physical map evidence.
        message.data = array('B', bytes(message.step*message.height))
        self.depth_pub.publish(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-clean', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
    os.environ['ELASTIC_TIMING_TRACE'] = str(args.output/'trace')
    config = yaml.safe_load((ROOT/'src/tool/navigation/planner/elastic_tracker/ros2_adapter/bridge/config/native_source.yaml').read_text())
    config['planner']['parameters']['source_driven_rpc'] = True
    (args.output/'native_source.yaml').write_text(yaml.safe_dump(config))
    rclpy.init()
    fixture = Inputs()
    fixture.enable_client = fixture.create_client(SetBool, '/drone0/elastic_adapter/enable')
    stop = fixture.create_client(Trigger, '/drone0/follow_timed_path/_behavior/stop')
    commands = [
        ['ros2','launch','uav_elastic_tracker_adapter','stack.launch.py',
         'camera_config:='+str(ROOT/'src/tool/navigation/planner/elastic_tracker/config/calibration-example.yaml'),
         'native_config:='+str(args.output/'native_source.yaml'), 'use_sim_time:=false',
         'depth_topic:=/synthetic/depth','fresh_native_output:=true'],
        ['ros2','run','uav_follow_path_execution','follow_path_extended_node','--ros-args',
         '--params-file',str(Path(old.get_package_share_directory('as2_behaviors_motion'))/'follow_path_behavior/config/config_default.yaml'),
         '-r','__ns:=/drone0','-p','plugin_name:=follow_path_plugin_position',
         '-p','follow_path_speed:=1.0','-p','follow_path_threshold:=0.2',
         '-p','tf_timeout_threshold:=0.1', '-p','timed_acceleration_continuity_tolerance:=1.4']]
    names = ['native', 'follow_timed_path']
    processes, logs = [], []
    report = dict(scope='real native planner and standard FollowTimedPath with synthetic sensors/controller')
    try:
        for name, command in zip(names, commands):
            stream = (args.output/(name+'.log')).open('w'); logs.append(stream)
            processes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
        end = time.monotonic()+25.
        while True:
            fixture.test_executor.spin_once(timeout_sec=.02)
            assert all(p.poll() is None for p in processes), 'process exited; inspect logs'
            if (stop.service_is_ready() and fixture.enable_client.service_is_ready()
                                 and fixture.states and fixture.states[-1]['input_ready']):
                break
            assert time.monotonic() < end, 'startup discovery expired'
        assert fixture.enable(True).success
        fixture.spin(6.)
        report.update(state=fixture.states[-1], actual_references=len(fixture.trajectory_references))
        report['passed'] = (not report['state']['error'] and report['state']['revision'] > 1
                            and report['actual_references'] > 0)
        if args.require_clean:
            assert report['passed'], report
    finally:
        if fixture.enable_client.service_is_ready():
            fixture.enable(False)
            fixture.wait_until(lambda: fixture.states and fixture.states[-1]['stop_confirmed'])
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
        for stream in logs: stream.close()
        (args.output/'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
        fixture.test_executor.remove_node(fixture)
        fixture.destroy_node(); fixture.test_executor.shutdown(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
