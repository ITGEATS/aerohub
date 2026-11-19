"""Offline physical regression gate; AirSim truth is scoring only, never control."""
import argparse
import json
from pathlib import Path


def evaluate(directory):
    directory = Path(directory)
    samples = json.loads((directory/'physics-samples.json').read_text())
    result = json.loads((directory/'flight-result.json').read_text())
    valid = [s for s in samples if 'pose' in s]
    ground = valid[0]['pose']['position']['z_val']
    hover = [s for s in valid if s['phase'] in ('hover', 'parked_observation', 'moving_observation')]
    heights = [ground-s['pose']['position']['z_val'] for s in hover]
    initial_collision = valid[0]['collision']['time_stamp']
    collisions = sorted({s['collision']['object_name'] for s in valid
                         if s['phase'] in ('takeoff', 'hover', 'parked_observation', 'moving_observation')
                         and s['collision']['time_stamp'] != initial_collision
                         and s['collision']['normal']['z_val'] > .5})
    checks = dict(native_takeoff=result['takeoff_success'],
                  stable_hover=result['hover_stable'], native_land=result['land_success'],
                  disarmed_ground=result['disarmed_ground'],
                  physical_hover_height=bool(heights) and all(2.7 <= h <= 3.3 for h in heights),
                  no_overhead_contact=not collisions)
    return dict(passed=all(checks.values()), checks=checks, hover_physics_samples=len(heights),
                physical_hover_height_range_m=[min(heights), max(heights)] if heights else None,
                overhead_collision_objects=collisions)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('probe_directory', type=Path)
    args = parser.parse_args()
    report = evaluate(args.probe_directory)
    (args.probe_directory.parent/'physical-evaluation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)
