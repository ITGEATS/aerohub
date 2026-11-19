"""Load and sample the pinned native Tracker without changing its model callbacks."""
import hashlib
import sys

import numpy as np


SECOND = 10**9


def boundary(axes, t):
    return tuple(np.asarray([getattr(axis, method)(t) for axis in axes])
                 for method in ('get_position', 'get_velocity', 'get_acceleration'))


def load_tracker(upstream, *, image_fps=30):
    root = upstream.resolve() / 'YOPO'
    sys.path.insert(0, str(root))
    from test_yopo_ros import YopoNet

    weight = root / 'saved/YOPO_1/epoch50.pth'
    digest = hashlib.sha256(weight.read_bytes()).hexdigest()
    if digest != 'a625490468cb8734867e51d2af2be68a8182bcbbbb1e0401b1d2a32e6ccd5f7f':
        raise ValueError(f'Unexpected checkpoint SHA256: {digest}')
    settings = dict(use_tensorrt=False, pitch_angle_deg=0, image_fps=image_fps,
                    simulation=True, conf_thresh=0.9, ekf_thresh=2.0,
                    plan_from_reference=True, verbose=False, visualize=False,
                    odom_topic='replay/odom', depth_topic='replay/depth',
                    rgb_topic='replay/rgb', ctrl_topic='replay/unused_reference')
    return YopoNet(settings, str(weight)), digest


def sample_yaw(knots, elapsed_ns):
    times, angles = zip(*knots)
    return float(np.interp(elapsed_ns, times, np.unwrap(angles)))


def frozen_yaw(tracker, axes, duration_ns):
    position, yaw, step = tracker.desire_pos, tracker.last_yaw, tracker.ctrl_dt
    knots = [(0, yaw)]
    step_ns = round(step * SECOND)
    try:
        for t in (*range(step_ns, duration_ns, step_ns), duration_ns):
            tracker.desire_pos = boundary(axes, t / SECOND)[0]
            tracker.ctrl_dt = (t - knots[-1][0]) / SECOND
            tracker.last_yaw, _ = tracker.desired_yaw()
            knots.append((t, tracker.last_yaw))
    finally:
        tracker.desire_pos, tracker.last_yaw, tracker.ctrl_dt = position, yaw, step
    return knots
