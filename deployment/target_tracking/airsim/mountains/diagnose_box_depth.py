"""Offline v1 box-depth audit; never supplies measurements to a running Tool.

Uses full saved RGB/depth and an actual Engine box from a replay report. Manual
image interpretation is separate from the numerical support predicates.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'src/tool/object_detection/ros2_adapter'))
from uav_object_detection.box_depth import locate_box


def distribution(values):
    values = values[np.isfinite(values)]
    return dict(count=int(values.size), quantiles_m=np.percentile(
        values, [0, 10, 25, 50, 75, 90, 100]).tolist()) if values.size else dict(count=0)


def audit(depth, box):
    h, w = depth.shape
    x1, y1 = np.maximum(np.floor(box[:2]).astype(int), 0)
    x2, y2 = np.minimum(np.ceil(box[2:]).astype(int), [w, h])
    dx, dy = (x2-x1)//4, (y2-y1)//4
    inner = np.zeros(depth.shape, bool)
    inner[y1+dy:y2-dy, x1+dx:x2-dx] = True
    ring = np.zeros(depth.shape, bool)
    ring[y1:y2, x1:x2] = True
    ring &= ~inner
    valid = np.isfinite(depth) & (depth > .2) & (depth < 50.)
    roi = inner & valid
    values = depth[roi]
    bins, counts = np.unique(np.floor(values/.25).astype(int), return_counts=True)
    supported = bins[counts >= max(5, .1*len(values))]
    seed = float((supported[0]+.5)*.25) if len(supported) else None
    selected = roi & (np.abs(depth-seed) <= .25) if seed is not None else np.zeros_like(roi)
    z = float(np.median(depth[selected])) if selected.any() else None
    cx, cy = (x1+x2)//2, (y1+y2)//2
    center = selected[max(y1,cy-max(1,dy//2)):min(y2,cy+max(1,dy//2)),
                      max(x1,cx-max(1,dx//2)):min(x2,cx+max(1,dx//2))]
    spread = float(np.diff(np.percentile(depth[selected], [10,90]))[0]) if selected.any() else None
    background = depth[ring & valid]
    predicates = dict(valid_support=bool(roi.sum() >= 20 and roi.sum() >= .6*inner.sum()),
        supported_seed=bool(len(supported)),
        selected_support=bool(selected.sum() >= 20 and selected.sum() >= .6*roi.sum()),
        selected_spread=bool(spread is not None and spread <= .5),
        center_support=bool(center.size and center.sum() >= .6*center.size),
        surrounding_separation=bool(background.size >= 20 and z is not None and np.median(background) >= z+.5))
    # Upper bound: can ANY half-metre interval satisfy 60%? No policy mutation.
    ordered = np.sort(values)
    sizes = np.searchsorted(ordered, ordered+.5, side='right')-np.arange(len(ordered))
    best = int(np.argmax(sizes))
    try:
        point, detail = locate_box(depth, box, [320.,320.,320.,240.], np.eye(3), np.zeros(3))
        actual = dict(accepted=True, point_optical=point.tolist(), detail=detail)
    except ValueError as exc:
        actual = dict(accepted=False, reason=str(exc))
    spatial = []
    for b, count in zip(bins, counts):
        yy, xx = np.nonzero(roi & (np.floor(depth/.25) == b))
        spatial.append(dict(bin_start_m=float(b*.25),count=int(count),fraction=float(count/values.size),
            median_uv=[float(np.median(xx)),float(np.median(yy))]))
    report = dict(box=list(map(float,box)), clipped_box=[int(x1),int(y1),int(x2),int(y2)],
        inner=distribution(values), ring=distribution(background), selected=distribution(depth[selected]),
        rejected=distribution(depth[roi & ~selected]), bins=spatial, seed_m=seed,
        selected_fraction=float(selected.sum()/roi.sum()), center_fraction=float(center.mean()),
        selected_spread_m=spread, predicates=predicates, actual_policy=actual,
        best_any_half_metre_window=dict(start_m=float(ordered[best]),
            end_m=float(ordered[best]+.5), fraction=float(sizes[best]/len(ordered))),
        note='Other predicates are counterfactual after first rejection; not accepted measurements.')
    return report, inner, selected


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir',type=Path,required=True)
    p.add_argument('--replay',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    rgb=a.input_dir/'hover-parked.png';source=a.input_dir/'hover-parked-depth.npy'
    depth=np.load(source,allow_pickle=False)
    replay=json.loads(a.replay.read_text())
    box=replay['engine_reference'][0]['xyxy']
    report,inner,selected=audit(depth,box)
    report['inputs']={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (rgb,source,a.replay)}
    report['scope']='saved real RGB-D offline audit; no live input, no target centroid or 3D accuracy claim'
    # Deliberately synthetic counterexample, distinct from the real scene.
    synthetic=np.full((40,40),8.)
    synthetic[10:30,10:30]=np.linspace(2.2,2.69,20)[None,:]
    counterexample,_,_=audit(synthetic,[0.,0.,40.,40.])
    counterexample['scope']='synthetic single continuous sloping foreground with separated background'
    report['synthetic_counterexample']=counterexample
    (a.output/'depth-audit.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from PIL import Image
    fig,axes=plt.subplots(2,2,figsize=(13,9))
    axes[0,0].imshow(Image.open(rgb));axes[0,0].set_title('Original full RGB / actual DINO box')
    axes[0,1].imshow(depth,vmin=2,vmax=6,cmap='viridis');axes[0,1].set_title('Full raw DepthPlanar (m); display range 2–6')
    for ax in axes[0]:
        x1,y1,x2,y2=report['clipped_box'];ax.add_patch(Rectangle((x1,y1),x2-x1,y2-y1,fill=False,edgecolor='red'))
        yy,xx=np.nonzero(inner);ax.add_patch(Rectangle((xx.min(),yy.min()),xx.max()-xx.min(),yy.max()-yy.min(),fill=False,edgecolor='cyan'))
    labels=np.zeros(depth.shape);labels[inner]=1;labels[selected]=2
    axes[1,0].imshow(labels,vmin=0,vmax=2,cmap='viridis');axes[1,0].set_title('Middle ROI: rejected purple/teal, selected yellow')
    axes[1,1].hist(depth[inner],bins=np.arange(2.5,4.26,.25),label='ROI')
    axes[1,1].hist(depth[selected],bins=np.arange(2.5,4.26,.25),label='selected')
    axes[1,1].set_xlabel('Optical depth (m)');axes[1,1].set_ylabel('Pixels');axes[1,1].legend()
    fig.tight_layout();fig.savefig(a.output/'depth-spatial.png',dpi=150);plt.close(fig)
    print(json.dumps({k:report[k] for k in ('actual_policy','selected_fraction','center_fraction','best_any_half_metre_window')}))


if __name__=='__main__':main()
