"""Explicit known-height ranging for a full, approximately upright person.

This is a pinhole size prior, not depth sensing or an Elastic-Tracker port.
The returned point approximates the vertical center of the person. Real running
poses change apparent height; target_height_m is a caller-supplied assumption.
"""
import math
import numpy as np


def locate_known_height(box, intrinsics, image_size, rotation, translation,
                        target_height_m, min_box_height_px=12., max_range_m=40.):
    values = np.asarray([*box, *intrinsics, *image_size, target_height_m,
                         min_box_height_px, max_range_m], dtype=float)
    r, t = np.asarray(rotation, dtype=float), np.asarray(translation, dtype=float)
    if (not np.isfinite(values).all() or r.shape != (3, 3) or t.shape != (3,) or
            not np.isfinite(r).all() or not np.isfinite(t).all() or
            not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or
            not math.isclose(np.linalg.det(r), 1., abs_tol=1e-5)):
        raise ValueError('invalid known-height geometry')
    x1, y1, x2, y2 = box
    fx, fy, cx, cy = intrinsics
    width, height = image_size
    if min(fx, fy, width, height, target_height_m, min_box_height_px, max_range_m) <= 0:
        raise ValueError('known-height calibration and height must be positive')
    if x2 <= x1 or y2-y1 < min_box_height_px:
        raise ValueError('person box too small for height ranging')
    if min(x1, y1) <= 1 or x2 >= width-1 or y2 >= height-1:
        raise ValueError('truncated person box; full height required')
    # A vertical segment of height H has endpoints c +/- (H/2)*up_camera.
    # Solve their normalized image y coordinates for center optical Z and Y.
    # For a level camera this reduces to Z = fy * H / box_height_pixels.
    up = r.T @ np.array([0., 0., 1.])
    if up[1] >= -.5 or abs(up[2]) > .8:
        raise ValueError('height prior requires an upright side/front view')
    top, bottom = (y1-cy)/fy, (y2-cy)/fy
    z = target_height_m * (up[1] - .5*up[2]*(top+bottom)) / (top-bottom)
    y = top*z + .5*target_height_m*(top*up[2]-up[1])
    # Box horizontal center approximates the projection of the vertical center.
    x = (((x1+x2)/2)-cx)*z/fx
    camera = np.array([x, y, z])
    distance = float(np.linalg.norm(camera))
    if z <= 0 or not np.isfinite(camera).all() or distance > max_range_m:
        raise ValueError('known-height range outside configured geometry')
    return r @ camera + t, dict(optical_z_m=float(z), range_m=distance,
                                assumed_height_m=float(target_height_m),
                                box_height_px=float(y2-y1))
