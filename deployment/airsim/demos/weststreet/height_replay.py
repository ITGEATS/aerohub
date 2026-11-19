#!/usr/bin/env python3
"""Size-prior diagnostics on recorded real YOLO results; no new inference/flight.

Recorded transforms belong to the paired depth timestamp, not necessarily RGB.
This replay is a range-method diagnostic, not live capture-time TF acceptance.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'src/tool/object_detection/ros2_adapter'))
from uav_object_detection.height_range import locate_known_height


def rotation(q):
    x,y,z,w = np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--height',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    source = json.loads(args.input.read_text())
    audits = {x['rgb_stamp_ns']:x for x in source['audits'] if 'camera_quaternion' in x}
    rows = []
    for sample in source['observations']:
        stamp = sample['rgb_stamp_ns']
        if stamp not in audits:
            rows.append(dict(rgb_stamp_ns=stamp,phase=sample['phase'],skipped='no paired calibration/TF audit'))
            continue
        audit = audits[stamp]
        k = audit['rgb_k']
        row = dict(rgb_stamp_ns=stamp,phase=sample['phase'],
                   tf_stamp_source='recorded paired depth capture',
                   tf_to_rgb_skew_s=(audit['depth_stamp_ns']-stamp)/1e9,estimates=[])
        for detection in sample['detections']:
            try:
                point, diagnostic = locate_known_height(detection['box'],[k[0],k[4],k[2],k[5]],
                    (640,480),rotation(audit['camera_quaternion']),audit['camera_position'],args.height)
                entry = dict(confidence=detection['confidence'],position=point.tolist(),**diagnostic,
                             native_depth_valid=detection['position_valid'],native_depth_error=detection['position_error'])
                if detection['position_valid']:
                    entry['relative_to_native_surface_z'] = diagnostic['optical_z_m']/detection['depth_m']-1
                row['estimates'].append(entry)
            except ValueError as exc:
                row['estimates'].append(dict(error=str(exc)))
        rows.append(row)
    result = dict(scope='real-image GPU detections replayed through CPU size-prior geometry; no live ROS or flight',
                  source=str(args.input),assumed_height_m=args.height,observations=rows)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2))
    for row in rows:
        print(row['phase'],row.get('skipped') or [(round(x['optical_z_m'],2),x['native_depth_valid']) for x in row['estimates'] if 'optical_z_m' in x])


if __name__=='__main__':
    main()
