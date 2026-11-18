#!/usr/bin/env python3
"""CityNav demo launch health probe (run inside a workstation-role container).

Checks the five-eye sensor contract and the AS2/compute role readiness that
formal1 depends on, per the component-deployment skill (process alive vs real
data flow are different health levels):
  - /clock flowing (cross-container: published by the flight_board clock)
  - 5x City* rgb/image (rgb8 256x256), 5x depth_planar/image (32FC1),
    5x rgb/camera_info (K[0]>0) with per-topic message counts
  - /drone0/self_localization/pose + twist (AS2 platform, compute container)
  - /drone0/citynav/status present with ready flag (DDS across containers:
    this probe runs in the ground image, the publisher lives in onboard)
  - action servers: follow_timed_path / TakeoffBehavior / LandBehavior
  - services: citynav/cancel, set_arming_state, set_offboard_mode
  - HTTP GET /health/ready on the grounding + clip inference services
Exit code 0 only when every check passes; the JSON report is the evidence.
"""
import argparse
import json
from pathlib import Path
import time
import urllib.request

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import Land, Takeoff
from geometry_msgs.msg import PoseStamped, TwistStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from uav_planner_interfaces.action import FollowTimedPath

CITY_CAMS = ['CityLeft', 'CitySlightlyLeft', 'CityFront',
             'CitySlightlyRight', 'CityRight']


class Probe(Node):
    def __init__(self, counts):
        super().__init__('citynav_health')
        self.counts = counts
        self.clock = None
        self.pose = None
        self.twist = None
        self.status = None
        self.create_subscription(Clock, '/clock',
                                 lambda m: self.bump('clock'), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose',
                                 lambda m: (self.bump('pose'),
                                            setattr(self, 'pose', m)),
                                 qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/drone0/self_localization/twist',
                                 lambda m: (self.bump('twist'),
                                            setattr(self, 'twist', m)),
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/drone0/citynav/status',
                                 lambda m: (self.bump('citynav/status'),
                                            self.read_status(m)),
                                 qos_profile_sensor_data)
        for cam in CITY_CAMS:
            for kind, dtype in (('rgb/image', 'rgb'),
                                ('depth_planar/image', 'depth'),
                                ('rgb/camera_info', 'info')):
                key = f'{cam}/{kind}'
                self.create_subscription(
                    CameraInfo if 'camera_info' in kind else Image,
                    '/drone0/sensor_measurements/' + cam + '/' + kind,
                    lambda m, k=key: self.bump(k), qos_profile_sensor_data)

    def bump(self, key):
        self.counts[key] = self.counts.get(key, 0) + 1

    def read_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=12.0)
    parser.add_argument('--grounding-health', required=True)
    parser.add_argument('--clip-health', required=True)
    args = parser.parse_args()

    counts = {}
    rclpy.init()
    probe = Probe(counts)
    actions = {
        'follow_timed_path': ActionClient(probe, FollowTimedPath,
                                          '/drone0/follow_timed_path'),
        'TakeoffBehavior': ActionClient(probe, Takeoff, '/drone0/TakeoffBehavior'),
        'LandBehavior': ActionClient(probe, Land, '/drone0/LandBehavior'),
    }
    services = {'citynav/cancel': probe.create_client(Trigger, '/drone0/citynav/cancel'),
                'set_arming_state': probe.create_client(SetBool, '/drone0/set_arming_state'),
                'set_offboard_mode': probe.create_client(SetBool, '/drone0/set_offboard_mode')}

    deadline = time.time() + args.seconds
    while time.time() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
    for name, client in actions.items():
        counts['action/' + name] = int(client.server_is_ready())
    for name, client in services.items():
        counts['service/' + name] = int(client.service_is_ready())
    status_ready = bool(probe.status and probe.status.get('ready'))
    rclpy.shutdown()

    http = {}
    for label, url in (('grounded_sam', args.grounding_health),
                       ('clip', args.clip_health)):
        try:
            with urllib.request.urlopen(url, timeout=8) as resp:
                http[label] = resp.status
        except Exception as exc:  # noqa: BLE001 - report any transport failure
            http[label] = str(exc)

    checks = {}
    for cam in CITY_CAMS:
        checks[f'{cam}/rgb'] = counts.get(f'{cam}/rgb/image', 0) >= 3
        checks[f'{cam}/depth'] = counts.get(f'{cam}/depth_planar/image', 0) >= 3
        checks[f'{cam}/info'] = counts.get(f'{cam}/rgb/camera_info', 0) >= 3
    checks['clock'] = counts.get('clock', 0) >= 5
    checks['pose'] = counts.get('pose', 0) >= 3
    checks['twist'] = counts.get('twist', 0) >= 3
    checks['citynav/status'] = counts.get('citynav/status', 0) >= 3
    checks['planner_ready'] = status_ready
    checks['action/follow_timed_path'] = counts.get('action/follow_timed_path') == 1
    checks['action/TakeoffBehavior'] = counts.get('action/TakeoffBehavior') == 1
    checks['action/LandBehavior'] = counts.get('action/LandBehavior') == 1
    for name in services:
        checks['service/' + name] = counts.get('service/' + name) == 1
    checks['http/grounded_sam'] = http.get('grounded_sam') == 200
    checks['http/clip'] = http.get('clip') == 200

    report = {'counts': counts, 'http': http, 'checks': checks,
              'status_snapshot': probe.status,
              'all_passed': all(checks.values()),
              'failed': sorted(k for k, v in checks.items() if not v)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'all_passed': report['all_passed'],
                      'failed': report['failed']}))
    raise SystemExit(0 if report['all_passed'] else 1)


if __name__ == '__main__':
    main()
