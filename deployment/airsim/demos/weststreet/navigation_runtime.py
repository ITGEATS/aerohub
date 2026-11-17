#!/usr/bin/env python3
"""Onboard SUPER deployment, separate from workstation mission."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--target-yaw', action='store_true', help='Enable SUPER target yaw planning and Adapter execution')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('weststreet_navigation_runtime', namespace='drone0',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = dict(odom=None, cloud=0, map=0)
    node.create_subscription(Odometry, 'sensor_measurements/odom',
        lambda m:state.update(odom=m), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, 'navigation/points',
        lambda m:state.update(cloud=state['cloud']+int(m.width*m.height>0)), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/rog_map/occ',
        lambda m:state.update(map=state['map']+int(m.width*m.height>0)), qos_profile_sensor_data)
    def map_sample(message):
        state['map'] += int(message.width*message.height > 0)
        (args.output/'map-sample.json').write_text(json.dumps(dict(width=message.width, height=message.height, bytes=len(message.data), count=state['map'])))
    node.create_subscription(PointCloud2, '/rog_map/inf_occ', map_sample, qos_profile_sensor_data)
    def status():
        (args.output/'startup-state.json').write_text(json.dumps(dict(clouds=state['cloud'], maps=state['map'], odom=state['odom'] is not None)))
    node.create_timer(1., status)
    children = []
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    def start(name, command):
        stream = (args.output/(name+'.log')).open('w')
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, process, stream))
    def wait(predicate, timeout):
        deadline = time.monotonic()+timeout
        while not predicate():
            if stopping:
                raise InterruptedError('Navigation stopped')
            for name, process, _ in children:
                if process.poll() is not None:
                    raise RuntimeError(name+' exited '+str(process.returncode))
            if time.monotonic()>deadline:
                raise TimeoutError('Navigation startup timeout')
            rclpy.spin_once(node, timeout_sec=.02)
    try:
        start('inputs', ['ros2','run','as2_navigation_inputs','navigation_inputs_node','--ros-args',
            '-r','__ns:=/drone0','-p','use_sim_time:=true','-p','cloud_topic:=sensor_measurements/lidar1/points'])
        wait(lambda:state['odom'] is not None and state['cloud']>=3, 90)
        # PX4 local origin may reset during startup. Register the map only
        # after the same disarmed settle interval used by the forest runtime.
        settled = node.get_clock().now().nanoseconds
        wait(lambda:(node.get_clock().now().nanoseconds-settled)/1e9 >= 10., 35)
        origin = state['odom'].pose.pose.position
        config = yaml.safe_load(Path('/repo/deployment/airsim/demos/super_forest/planner.yaml').read_text())
        if args.target_yaw:
            config['fsm']['click_yaw_en'] = True
            config['super_planner']['goal_yaw_en'] = True
        # Only visualization bandwidth changes; planning/map geometry stays unchanged.
        config['rog_map']['visualization']['range'] = [12., 12., 8.]
        config['rog_map']['visualization']['time_rate'] = 1.
        config['rog_map']['fix_map_origin'] = [origin.x,origin.y,origin.z+3.]
        # ROGMap ORIGIN_AT_CORNER inflates the virtual floor upward
        # by inflation_step*resolution; preserve .3m center clearance.
        inflation = config['rog_map']['inflation_step']*config['rog_map']['inflation_resolution']
        config['rog_map']['virtual_ground_height'] = origin.z-inflation-.3
        config['rog_map']['virtual_ceil_height'] = origin.z+12.
        # Existing SUPER speed envelope; person speed is 5m/s.
        path = args.output/'planner.yaml'
        path.write_text(yaml.safe_dump(config))
        start('super', ['ros2','run','super_planner','fsm_node','--ros-args',
            '-p','use_sim_time:=true','-p','config_name:='+str(path)])
        start('adapter', ['ros2','run','uav_super_adapter','super_timed_path_adapter','--ros-args',
            '-r','__ns:=/drone0','-p','use_sim_time:=true','-p','navigation_action:=true','-p','target_yaw_enabled:='+str(args.target_yaw).lower()])
        wait(lambda:state['map']>=3, 60)
        (args.output/'ready.json').write_text(json.dumps(dict(ready=True, origin=[origin.x,origin.y,origin.z])))
        wait(lambda:False, 600)
    except InterruptedError:
        pass
    finally:
        for _, process, _ in children:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for _, process, stream in children:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            stream.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
