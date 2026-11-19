"""Compare the native EKF clock rates against identical recorded observations."""
import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import rclpy

from native_replay import load_tracker, replay


TOPICS = ['/drone_0/odom', '/drone_0/depth', '/drone_0/rgb']


def compare(upstream, bag, fps):
    tracker, digest = load_tracker(upstream, image_fps=fps)
    tracker.timer_ctrl.cancel()
    module = sys.modules[type(tracker).__module__]
    rows = []
    try:
        if tracker.device != 'cuda':
            raise RuntimeError('The rate comparison requires CUDA')

        def observe(depth, rgb):
            stamp = depth.header.stamp
            capture_ns = stamp.sec * 10**9 + stamp.nanosec
            previous = tracker.optimal_poly_x
            # Only the upstream callback's wall-time hover window is replayed on capture time.
            # Time-based EKF prediction still uses the specified native fixed step.
            with patch.object(module, 'time', SimpleNamespace(time=lambda: capture_ns / 1e9)):
                tracker.callback_images(depth, rgb)
            target = tracker.ekf_target
            rows.append(dict(capture_ns=capture_ns, confirmed=target is not None,
                             target_m=None if target is None else target.tolist(),
                             tracked_revision=tracker.optimal_poly_x is not previous and
                             tracker.last_trajectory_tracking and not tracker.searching))

        counts = replay(tracker, bag, 'sqlite3', TOPICS, on_images=observe)
        return dict(image_fps=fps, prediction_step_s=tracker.ekf_dt,
                    model_device=tracker.device, weight_sha256=digest, **counts, rows=rows)
    finally:
        tracker.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    rclpy.init(args=[])
    try:
        baseline, matched = (compare(args.upstream, args.bag, fps) for fps in (30, 15))
    finally:
        rclpy.shutdown()
    rows = baseline['rows'], matched['rows']
    stamps = [[row['capture_ns'] for row in run] for run in rows]
    if stamps[0] != stamps[1] or len(stamps[0]) < 2:
        raise RuntimeError('The two rates must consume identical observations')
    joint = [float(np.linalg.norm(np.subtract(a['target_m'], b['target_m'])))
             for a, b in zip(*rows) if a['confirmed'] and b['confirmed']]
    summary = dict(pairs=len(stamps[0]),
                   measured_rate_hz=(len(stamps[0]) - 1) * 1e9 / (stamps[0][-1] - stamps[0][0]),
                   confirmed_30=baseline['confirmed'], confirmed_15=matched['confirmed'],
                   plans_30=sum(row['tracked_revision'] for row in rows[0]),
                   plans_15=sum(row['tracked_revision'] for row in rows[1]),
                   maximum_common_estimate_delta_m=max(joint) if joint else None,
                   common_confirmed_pairs=len(joint))
    with args.output.open('x', encoding='utf-8') as output:
        json.dump(dict(summary=summary, runs=[baseline, matched]), output, indent=2)
        output.write('\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
