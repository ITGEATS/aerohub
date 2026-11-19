#!/usr/bin/env python3
"""Replay actual session observations against unchanged configured tracking; no truth."""
import json
from pathlib import Path
import sys
import numpy as np
from uav_target_tracking.core import Tracker

root=Path(sys.argv[1])
observations=json.loads((root/'filter-observations.json').read_text())
settings=json.loads((root/'predeclared.json').read_text()).get('tracking',{})
policy=settings.get('association_policy','innovation_covariance_v1')
tracker=Tracker(measurement_stddev_m=settings.get('measurement_stddev_m',1.),
    association_policy=policy,position_filter=settings.get('position_filter','robust_linear_v1'))
original=tracker.ekf.check_valid
rows=[]
def inspect(points):
    residual=points-tracker.ekf.x[:3]
    innovation=tracker.ekf.Sigma[:3,:3]+tracker.ekf.Rt
    scores=np.einsum('ij,ij->i',residual,np.linalg.solve(innovation,residual.T).T)
    rows[-1].update(predicted=tracker.ekf.x.tolist(),distances=np.linalg.norm(residual,axis=1).tolist(),
                    innovation_scores=scores.tolist())
    return original(points)
tracker.ekf.check_valid=inspect
for observation in observations:
    rows.append(dict(stamp=observation['stamp'],raw=observation['raw']))
    status=tracker.observe(observation['raw'],observation['stamp'],observation['frame'])
    rows[-1].update(status=status,filtered=tracker.measurement_audit['filtered'],
                    posterior=tracker.ekf.x.tolist(),blind_s=tracker.ekf.blind_time,
                    update=tracker.measurement_audit.get('association',{}))
report=dict(scope='Recorded session replay; no truth, no parameter changes, not flight proof',
            association_policy=policy,distance_prior_m=tracker.ekf.gate_max,rows=rows)
(root/'association-audit.json').write_text(json.dumps(report,indent=2))
for row in rows:
    print(json.dumps({k:row[k]for k in ('stamp','status','predicted','distances','innovation_scores','blind_s')if k in row}))
