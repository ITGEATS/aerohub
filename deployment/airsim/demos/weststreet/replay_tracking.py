#!/usr/bin/env python3
"""Real saved measurement replay, not flight evidence. No truth input."""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np
from uav_target_tracking.core import Tracker


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('calls',type=Path)
    p.add_argument('--measurement-stddev',type=float)
    p.add_argument('--innovation-association',action='store_true')
    p.add_argument('--position-filter',default='none')
    a=p.parse_args()
    root=Path(__file__).resolve().parents[4]
    path=root/'deployment/airsim/demos/target_tracking/reference/yopo_tracker/YOPO/policy/target_ekf.py'
    spec=importlib.util.spec_from_file_location('locked_ekf',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    tracker=Tracker(module.EKF, measurement_stddev_m=a.measurement_stddev,
        association_policy='innovation_covariance_v1' if a.innovation_association else 'upstream', position_filter=a.position_filter)
    confirmed=False;failures=[];rows=[]
    for line in a.calls.read_text().splitlines():
        item=json.loads(line)
        points=[d['position'] for d in item.get('detections',[]) if d['position_valid']]
        stamp=item['stamp_ns']/1e9
        status=tracker.observe(points,stamp,'drone0/odom')
        valid=tracker.snapshot(item.get('returned_ros_time',stamp))[1]
        if confirmed and not valid:failures.append(stamp)
        confirmed=confirmed or valid
        rows.append(dict(stamp=stamp,status=status,valid=valid,position=tracker.ekf.x[:3].tolist(),
                         blind=tracker.ekf.blind_time))
    print(json.dumps(dict(confirmed=confirmed,failures=failures,rows=rows),indent=2))
    if not confirmed or failures:raise SystemExit(1)


if __name__=='__main__':main()
