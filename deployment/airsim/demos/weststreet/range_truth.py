#!/usr/bin/env python3
"""Independent read-only truth recording and horizontal height-range scoring."""
import argparse
import bisect
import json
import math
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))


def rotate(q, v):
    x, y, z, w = (q[k] for k in ('x', 'y', 'z', 'w'))
    return [
        (1-2*(y*y+z*z))*v[0]+2*(x*y-z*w)*v[1]+2*(x*z+y*w)*v[2],
        2*(x*y+z*w)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-x*w)*v[2],
        2*(x*z-y*w)*v[0]+2*(y*z+x*w)*v[1]+(1-2*(x*x+y*y))*v[2]]


def record(path):
    from projectairsim import ProjectAirSimClient
    from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient
    client = ProjectAirSimClient(port_topics=19889, port_services=19890)
    client.connect_services()
    client.socket_services.recv_timeout = 3000
    actor = ProjectAirSimExtensionClient(port=19891, scene_port=19892)
    def rpc(method):
        return client.request(dict(method='/Sim/SceneWestStreetPx4/'+method, params={}, version=1.0))
    try:
        with path.open('w', buffering=1) as stream:
            while True:
                before = rpc('GetSimTime')
                runner = actor.get_actor_state()
                kin = rpc('robots/Drone1/GetGroundTruthKinematics')
                after = rpc('GetSimTime')
                stream.write(json.dumps(dict(before_ns=before, after_ns=after, actor=runner, kinematics=kin))+'\n')
                time.sleep(.025)
    finally:
        client.disconnect()


def score(path, evidence):
    samples = [json.loads(line) for line in path.read_text().splitlines()]
    times = [(s['before_ns']+s['after_ns'])/2 for s in samples]
    result = json.loads(evidence.read_text())
    audits = {a['rgb_stamp_ns']:a for a in result['audits']}
    rows = []
    for obs in result['observations']:
        stamp = obs['rgb_stamp_ns']
        row = dict(phase=obs['phase'], rgb_stamp_ns=stamp)
        detections = [d for d in obs['detections'] if d['category']=='person']
        audit = audits.get(stamp, {})
        i = bisect.bisect_left(times, stamp)
        if not detections or 'camera_position' not in audit or i==0 or i==len(times):
            row['skipped'] = 'missing detection, camera transform, or bracketing truth'
            rows.append(row)
            continue
        primary = max(detections, key=lambda d:d['confidence'])
        if not primary['position_valid']:
            row['skipped'] = primary['position_error']
            rows.append(row)
            continue
        pair = samples[i-1:i+1]
        # Reject intervals spanning reset/start/stop or a long recorder gap.
        span = (times[i]-times[i-1])/1e9
        if span>.15 or any(s['actor']['state']!=obs['phase'] for s in pair):
            row['skipped'] = 'truth gap >150ms or actor state transition'
            rows.append(row)
            continue
        fraction = (stamp-times[i-1])/(times[i]-times[i-1])
        distances = []
        for sample in pair:
            pose = sample['kinematics']['pose']
            # Camera FRD extrinsic from this probe's declared FrontCamera config.
            offset = rotate(pose['orientation'], [.6, 0, -.8])
            camera = [pose['position'][axis]+offset[k] for k,axis in enumerate('xyz')]
            target = sample['actor']['location_ue_cm']
            # UE centimetres NEU -> native global NED metres; horizontal axes identical.
            distances.append(math.hypot(target['x']/100-camera[0],target['y']/100-camera[1]))
        truth = distances[0]+fraction*(distances[1]-distances[0])
        estimate = math.hypot(*(primary['position'][k]-audit['camera_position'][k] for k in (0,1)))
        error = estimate-truth
        row.update(estimated_horizontal_m=estimate, true_horizontal_m=truth,
                   error_m=error, relative_error=error/truth, confidence=primary['confidence'],
                   truth_bracket_s=span, truth_rpc_span_s=max((s['after_ns']-s['before_ns'])/1e9 for s in pair))
        rows.append(row)
    phases = {}
    for phase in ('paused','running'):
        unique = {r['rgb_stamp_ns']:r for r in rows if r['phase']==phase and 'error_m' in r}
        valid = list(unique.values())
        phases[phase] = dict(samples=len(valid))
        if valid:
            phases[phase].update(mean_abs_error_m=statistics.mean(abs(r['error_m']) for r in valid),
                                 median_abs_relative_error=statistics.median(abs(r['relative_error']) for r in valid),
                                 max_abs_relative_error=max(abs(r['relative_error']) for r in valid))
        # Engineering sanity gate fixed before collecting this run.
        phases[phase]['sanity_passed'] = len(valid)>=4 and phases[phase]['median_abs_relative_error']<=.20 and phases[phase]['max_abs_relative_error']<=.30
    output = dict(assumed_height_m=result['assumed_height_m'], metric='camera to actor horizontal distance',
                  timing='linear interpolation of independent sim-clock-bracketed RPC truth; not atomic capture truth',
                  phases=phases, rows=rows)
    (path.parent/'range-check.json').write_text(json.dumps(output,indent=2))
    print(json.dumps(output,indent=2))


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record',type=Path)
    parser.add_argument('--score',type=Path)
    parser.add_argument('--evidence',type=Path)
    args = parser.parse_args()
    if args.record:
        record(args.record)
    else:
        score(args.score,args.evidence)
