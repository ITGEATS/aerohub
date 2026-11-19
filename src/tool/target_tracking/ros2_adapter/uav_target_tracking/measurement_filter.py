"""Opt-in single-candidate, causal robust short-window position measurement filter.

World-frame coordinates only. A local linear trend is evaluated at the ORIGINAL
capture stamp: unlike a trailing mean this preserves constant-velocity motion.
This is preprocessing, not a new observation or identity tracker.
"""
from collections import deque
import math
import numpy as np


class ShortWindowFilter:
    def __init__(self):
        self.history=deque(maxlen=5)
        self.last_input=None
        self.last_status='empty'

    def trend(self, samples, stamp):
        times=np.array([t-stamp for t,_ in samples])
        points=np.array([p for _,p in samples])
        slopes=[(points[j]-points[i])/(times[j]-times[i])
                for i in range(len(samples)) for j in range(i+1,len(samples))
                if times[j]-times[i]>=.04]
        velocity=np.median(slopes,axis=0) if slopes else np.zeros(3)
        speed=np.linalg.norm(velocity)
        if speed>10.:velocity=velocity*(10./speed)
        aligned=points-times[:,None]*velocity
        median=np.median(aligned,axis=0)
        mad=np.median(np.abs(aligned-median),axis=0)
        radius=np.maximum(.15,2.5*1.4826*mad)
        return np.mean(np.clip(aligned,median-radius,median+radius),axis=0)

    def apply(self, positions, stamp):
        points=np.asarray(positions,dtype=float).reshape((-1,3))
        if not math.isfinite(stamp) or stamp<=0 or not np.isfinite(points).all():
            raise ValueError('invalid filter measurement')
        if self.last_input is not None and stamp<=self.last_input:
            raise ValueError('duplicate or out-of-order filter measurement')
        self.last_input=stamp
        if self.history and stamp-self.history[-1][0]>.5:
            self.history.clear()
        while self.history and stamp-self.history[0][0]>.8:
            self.history.popleft()
        if len(points)==0:
            self.last_status='no_measurement'
            return points  # no hold-last-value disguised as a fresh detection
        if len(points)!=1:
            # Ambiguous candidates never enter the smoother. Preserve recent
            # history: clearing it makes a short noisy warmup define a false
            # velocity and reject subsequent motion. Normal expiry still applies.
            self.last_status='multiple_candidates_bypass'
            return points  # let existing association decide; never blend people
        if len(self.history)>=3:
            predicted=self.trend(list(self.history),stamp)
            if np.linalg.norm(points[0]-predicted)>=3.:
                self.last_status='jump_rejected'
                return np.empty((0,3))
        self.history.append((stamp,points[0].copy()))
        self.last_status='smoothed' if len(self.history)>=3 else 'warming_up'
        if len(self.history)<3:return points.copy()
        return self.trend(list(self.history),stamp).reshape((1,3))
