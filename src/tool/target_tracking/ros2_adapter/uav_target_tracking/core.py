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

class InnovationAssociation:
    """Adapter policy for an explicitly declared measurement noise source.

    Nominal 3D chi-square(0.99) gating uses predicted AND measurement covariance.
    The locked EKF hard distance cap remains enforced. This is a model assumption,
    not a calibrated probability guarantee or replacement for identity tracking.
    """
    def check_valid(self, points):
        points = np.atleast_2d(points)
        residual = points-self.x[:3]
        innovation = self.Sigma[:3,:3]+self.Rt
        scores = np.einsum('ij,ij->i', residual, np.linalg.solve(innovation, residual.T).T)
        distances = np.linalg.norm(residual, axis=1)
        ids = np.flatnonzero((scores <= 11.344866730144373) & (distances < self.gate_max))
        best = ids[np.argmin(scores[ids])] if ids.size else np.argmin(scores)
        return ids, int(best)


class SoftInnovationAssociation:
    """Robust source-aware updates; distance is a soft weight within reach.

    Reach uses the existing speed/blind-time budget, rather than the old 3m
    rejection alone. Identity remains geometric, not appearance tracking.
    """
    def check_valid(self, points):
        points = np.atleast_2d(points)
        residual = points-self.x[:3]
        innovation = self.Sigma[:3,:3]+self.Rt
        scores = np.einsum('ij,ij->i', residual, np.linalg.solve(innovation,residual.T).T)
        reach = self.gate_max+self.target_max_speed*getattr(self,"association_blind_time",self.max_blind_time)
        ids = np.flatnonzero(np.linalg.norm(residual,axis=1)<reach)
        best = int(ids[np.argmin(scores[ids])]) if ids.size else int(np.argmin(scores))
        # Multiple equally plausible candidates are not evidence of identity.
        if ids.size>1:
            ordered=np.sort(scores[ids])
            if ordered[1]-ordered[0]<1.:
                ids=np.empty(0,dtype=int)
        self.update_audit=dict(candidate_count=len(points),reachable_count=len(ids),
            residual_m=float(np.linalg.norm(residual[best])),reach_m=reach,
            status='associated' if ids.size else 'unmatched')
        return ids,best

    def update(self, z):
        residual=z-self.C@self.x
        innovation=self.C@self.Sigma@self.C.T+self.Rt
        score=float(residual@np.linalg.solve(innovation,residual))
        weight=min(1.,2.5/max(math.sqrt(max(0.,score)),1e-9))
        noise=self.Rt/(weight*weight)
        gain=np.linalg.solve((self.C@self.Sigma@self.C.T+noise).T,
                             (self.Sigma@self.C.T).T).T
        correction=gain@residual
        # Bound a single measurement's effect without throwing it away.
        pos_limit=self.target_max_speed*self.dt
        vel_limit=self.target_max_speed/getattr(self,"association_blind_time",self.max_blind_time)*self.dt
        scale=min(1.,pos_limit/max(np.linalg.norm(correction[:3]),1e-9),
                     vel_limit/max(np.linalg.norm(correction[3:]),1e-9))
        gain*=scale;correction*=scale
        self.K=gain;self.x=self.x+correction
        identity=np.eye(6)-gain@self.C
        self.Sigma=identity@self.Sigma@identity.T+gain@noise@gain.T
        self.update_audit.update(status='soft_update' if weight<1. or scale<1. else 'update',
            weight=weight,correction_scale=scale,position_correction_m=float(np.linalg.norm(correction[:3])),
            velocity_correction_mps=float(np.linalg.norm(correction[3:])))


class Tracker:
    def __init__(self, ekf_class=None, dt=0.1, *, measurement_stddev_m=None, association_policy="upstream", position_filter="none", max_prediction_time=None):
        if association_policy not in ('upstream', 'innovation_covariance_v1', 'soft_innovation_v1'):
            raise ValueError('unknown association_policy')
        base = ekf_class or load_ekf()
        if association_policy != 'upstream':
            if measurement_stddev_m is None:
                raise ValueError('covariance association requires an explicit noise source')
            policy = SoftInnovationAssociation if association_policy == "soft_innovation_v1" else InnovationAssociation
            class SourceAwareEKF(policy, base):
                pass
            base = SourceAwareEKF
        self.ekf = base(dt=dt)
        self.ekf.association_blind_time=self.ekf.max_blind_time
        if max_prediction_time is not None:
            if not math.isfinite(max_prediction_time) or not .5<=max_prediction_time<=2.:
                raise ValueError('max_prediction_time must be within .5..2 seconds')
            self.ekf.max_blind_time=max_prediction_time
        if measurement_stddev_m is not None:
            if not math.isfinite(measurement_stddev_m) or measurement_stddev_m <= 0:
                raise ValueError('measurement_stddev_m must be finite and positive')
            # Explicit source model, not a new association/expiry threshold.
            # None preserves the locked upstream covariance exactly.
            self.ekf.Rt = np.eye(3)*measurement_stddev_m**2
        if position_filter not in ('none', 'robust_linear_v1'):
            raise ValueError('unknown position_filter')
        self.measurement_filter = None
        if position_filter != 'none':
            from .measurement_filter import ShortWindowFilter
            self.measurement_filter = ShortWindowFilter()
        self.measurement_audit = {}
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
        raw = points.copy()
        if self.measurement_filter is not None:
            points = self.measurement_filter.apply(points, stamp)
        self.measurement_audit = dict(stamp=stamp, frame=frame,
            policy='robust_linear_v1' if self.measurement_filter else 'none',
            status=self.measurement_filter.last_status if self.measurement_filter else 'raw',
            raw=raw.tolist(), filtered=points.tolist())
        if hasattr(self.ekf,"update_audit"):self.ekf.update_audit={}
        ids, _, confirmed = self.ekf.predict_and_update(points)
        self.measurement_audit["association"] = getattr(self.ekf,"update_audit",{})
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
