"""Conservative visible foreground surface measurement, never an object centroid.

No segmentation is inferred. Background separation is required; a flat ROI alone
cannot distinguish an object from the background behind an inaccurate box.
"""
import numpy as np


def locate_box(depth, box, k, rotation, translation):
    if depth.ndim != 2 or not np.isfinite(box).all() or len(box) != 4:
        raise ValueError('invalid box/depth')
    h, w = depth.shape
    x1, y1 = np.maximum(np.floor(box[:2]).astype(int), 0)
    x2, y2 = np.minimum(np.ceil(box[2:]).astype(int), [w, h])
    if x2-x1 < 8 or y2-y1 < 8:
        raise ValueError('insufficient box support after clipping')
    # Middle half used for foreground; outer ring used to reject ambiguity.
    dx, dy = (x2-x1)//4, (y2-y1)//4
    inner = np.zeros(depth.shape, bool)
    inner[y1+dy:y2-dy, x1+dx:x2-dx] = True
    ring = np.zeros(depth.shape, bool)
    ring[y1:y2, x1:x2] = True
    ring &= ~inner
    valid = np.isfinite(depth) & (depth > .2) & (depth < 50.)
    roi = inner & valid
    if roi.sum() < 20 or roi.sum() < .6*inner.sum():
        raise ValueError('insufficient valid foreground depth')
    values = depth[roi]
    # Choose the nearest supported bin, not the most numerous background bin.
    bins, counts = np.unique(np.floor(values/.25).astype(int), return_counts=True)
    supported = bins[counts >= max(5, .1*len(values))]
    if not len(supported):
        raise ValueError('no supported foreground depth cluster')
    seed = (supported[0]+.5)*.25
    selected = roi & (np.abs(depth-seed) <= .25)
    if selected.sum() < 20 or selected.sum() < .6*roi.sum():
        raise ValueError('ambiguous competing foreground depths')
    z = float(np.median(depth[selected]))
    spread = float(np.percentile(depth[selected], 90)-np.percentile(depth[selected], 10))
    if spread > .5:
        raise ValueError('unreliable foreground depth spread')
    cx, cy = (x1+x2)//2, (y1+y2)//2
    center = selected[max(y1,cy-max(1,dy//2)):min(y2,cy+max(1,dy//2)),
                      max(x1,cx-max(1,dx//2)):min(x2,cx+max(1,dx//2))]
    if not center.size or center.sum() < .6*center.size:
        raise ValueError('foreground lacks center support')
    background = depth[ring & valid]
    if background.size < 20 or float(np.median(background)) < z+.5:
        raise ValueError('background ambiguity: no separated surrounding depth')
    yy, xx = np.nonzero(selected)
    u, v = float(np.median(xx)), float(np.median(yy))
    fx, fy, px, py = k
    camera = np.array([(u-px)*z/fx, (v-py)*z/fy, z])
    point = rotation @ camera + translation
    if not np.isfinite(point).all():
        raise ValueError('nonfinite surface projection')
    return point, dict(depth_m=z, spread_m=spread, valid_pixels=int(selected.sum()))
