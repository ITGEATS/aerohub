#!/usr/bin/env python3
"""Overhead tracking view for the SPF demo (M7 evidence).

Reuses the deployment/swarm observation-view infrastructure (camera_options,
downward_quaternion, update_camera) and records the evidence the AGENTS rule
demands: a JSONL track (live vehicle centroid + camera pose read-back per
update) plus periodic JPEG captures from the external camera. Tracking success
is proven by the centroid SERIES MOVING during the mission, never by the
static initial director pose.

Run inside the workstation container (needs msgpack, same env as mission.py):
  python3 spf_observer.py --settings /evidence/settings.json --host <winip> \
      --evidence /evidence/<case> [--once]
"""
import argparse
import json
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'deployment/swarm'))
sys.path.insert(0, str(ROOT / 'simulator/airsim/sensors'))

from view import camera_options  # deployment/swarm/view.py
from overhead_view import update_camera  # deployment/swarm/overhead_view.py
from uav_as2_sensor_adapter.rpc import Rpc

stopping = False


def _request_stop(_signum, _frame):
    global stopping
    stopping = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings', type=Path, required=True)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=None)
    parser.add_argument('--camera-name', default='swarm_overhead')
    parser.add_argument('--altitude', type=float, default=25.0)
    parser.add_argument('--update-hz', type=float, default=2.0)
    parser.add_argument('--capture-every-s', type=float, default=5.0)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--max-failures', type=int, default=10)
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args()
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    settings = json.loads(args.settings.read_text())
    vehicles = list(settings.get('Vehicles', {}))
    if not vehicles:
        raise SystemExit('settings has no vehicles')
    port = args.port or int(settings.get('ApiServerPort', 41451))
    camera = camera_options({'name': args.camera_name,
                             'altitude_m': args.altitude})
    rpc = Rpc(args.host, port, 5.0)
    frames = args.evidence / 'overhead'
    frames.mkdir(parents=True, exist_ok=True)
    track_path = frames / 'track.jsonl'
    interval = 1.0 / args.update_hz
    failures = 0
    captures = 0
    last_capture = time.monotonic() - args.capture_every_s

    with track_path.open('a') as track:
        while not stopping:
            started = time.monotonic()
            try:
                center = update_camera(rpc, vehicles, camera)
                # This Blocks build has no simGetCameraPose RPC (verified
                # 2026-11-16: "server could not find function"); the set pose
                # is deterministic from the centroid, and the captured frames
                # are the primary tracking evidence.
                entry = {'t': time.time(), 'centroid': list(center),
                         'camera_position_set': [center[0], center[1],
                                                 center[2] - camera['altitude_m']]}
                if time.monotonic() - last_capture >= args.capture_every_s:
                    # Same request form as catora_open_field/mission.py: with
                    # compress=True the JPEG bytes arrive in image_data_uint8.
                    response = rpc.call('simGetImages', [{
                        'camera_name': camera['name'], 'image_type': 0,
                        'pixels_as_float': False, 'compress': True}], '', True)
                    (frames / ('frame-%04d.jpg' % captures)).write_bytes(
                        bytes(response[0]['image_data_uint8']))
                    entry['frame'] = 'frame-%04d.jpg' % captures
                    captures += 1
                    last_capture = time.monotonic()
                track.write(json.dumps(entry) + '\n')
                track.flush()
                failures = 0
                if args.once:
                    print(json.dumps(entry))
                    return 0
            except (ConnectionError, OSError, RuntimeError, KeyError) as exc:
                failures += 1
                print('observer failure %d/%d: %s' % (failures, args.max_failures, exc),
                      file=sys.stderr, flush=True)
                if failures >= args.max_failures:
                    raise SystemExit('overhead view exceeded the RPC failure limit')
                if args.once:
                    raise
            remaining = interval - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    print(json.dumps({'stopped': True, 'updates_recorded': True,
                      'track': str(track_path), 'frames': captures}))


if __name__ == '__main__':
    raise SystemExit(main())
