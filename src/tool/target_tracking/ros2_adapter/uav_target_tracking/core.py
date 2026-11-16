"""Thin variable-observation-step adapter around the unchanged locked EKF."""
import importlib.util
import math
import numpy as np
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def load_ekf():
    path = Path(get_package_share_directory('uav_target_tracking')) / 'upstream/target_ekf.py'
    spec = importlib.util.spec_from_file_location('yopo_tracker_target_ekf', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.EKF

class Tracker:
    def __init__(self, ekf_class=None, dt=0.1):
        self.ekf = (ekf_class or load_ekf())(dt=dt)
        self.stamp = None
        self.observation_stamp = None
        self.frame = None

    def observe(self, positions, stamp, frame):
        points = np.asarray(positions, dtype=float)
        if points.size == 0:
            points = np.empty((0, 3))
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('positions must have shape (N, 3)')
        if not math.isfinite(stamp) or stamp <= 0 or not frame or not np.isfinite(points).all():
            raise ValueError('invalid measurement')
        if self.frame is not None and frame != self.frame:
            raise ValueError('frame changed')
        if self.stamp is not None:
            dt = stamp - self.stamp
            if dt <= 0:
                raise ValueError('duplicate or out-of-order capture')
            # Upstream dt also drives blind/reachable-set timers. Update all dt
            # dependent matrices before invoking the original algorithm.
            self.ekf.dt = dt
            self.ekf.A[:3, 3:] = np.eye(3) * dt
            self.ekf.B[:3] = np.eye(3) * (dt ** 2 / 2)
            self.ekf.B[3:] = np.eye(3) * dt
        self.stamp, self.frame = stamp, frame
        ids, _, confirmed = self.ekf.predict_and_update(points)
        if self.ekf.blind_time == 0:
            self.observation_stamp = stamp
        return ('tracked' if ids.size else 'predicted') if confirmed else ('lost' if self.ekf.blind_time > self.ekf.max_blind_time else 'confirm')

    def snapshot(self, now):
        if self.stamp is None:
            return 'unavailable', False, np.zeros(6), np.zeros((6, 6))
        age = now - self.stamp
        valid = not self.ekf.lost and age >= 0 and age + self.ekf.blind_time <= self.ekf.max_blind_time
        x, cov = self.ekf.x.copy(), self.ekf.Sigma.copy()
        if valid and age:
            a = np.eye(6); a[:3, 3:] = np.eye(3) * age
            b = np.vstack((np.eye(3) * age ** 2 / 2, np.eye(3) * age))
            x = a @ x; cov = a @ cov @ a.T + b @ self.ekf.Qt @ b.T
        status = 'predicted' if valid else ('lost' if age + self.ekf.blind_time > self.ekf.max_blind_time else 'confirm')
        return status, valid, x, cov
