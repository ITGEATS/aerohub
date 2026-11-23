"""Stage1 traj_server consumer semantics; no suffixing or execution admission."""
import math
from .core import evaluate, stamp_ns

# stage1/planning/src/traj_server.cpp:164 and :101, stage2 drone_consumer=true.
PERIOD = .01
YAW_STEP = .2


class NativeConsumer:
    def __init__(self):
        self.latest = self.previous = None
        self.yaw = 0.  # reference traj_server.cpp last_yaw_ initialization
        self.replaced = 0

    def replace(self, native):
        if not math.isfinite(native['yaw']):
            raise ValueError('nonfinite yaw')
        if native['hover']:
            if len(native['hover_p'])!=3 or not all(map(math.isfinite,native['hover_p'])):
                raise ValueError('invalid hover')
        elif (native['order']!=5 or not native['duration'] or
              any(not math.isfinite(d) or d<=0 for d in native['duration']) or
              any(len(native[k])!=len(native['duration'])*6 or not all(map(math.isfinite,native[k]))
                  for k in ('coef_x','coef_y','coef_z'))):
            raise ValueError('invalid native quintic')
        if self.latest is not None:
            self.replaced += 1
        self.latest = native
        if self.previous is None:
            self.previous = native

    def execute(self, native, now_ns):
        if native is None:
            return None
        t = (now_ns-stamp_ns(native['start_time']))/1e9
        if t <= 0:
            return None
        if native['hover']:
            if len(native['hover_p']) != 3:
                raise ValueError('hover requires three coordinates')
            self.yaw = native['yaw']  # stage1 drone_consumer hover rule
            return [native['hover_p'], [0.,0.,0.], [0.,0.,0.]], self.yaw
        if native['order'] != 5 or any(len(native[k]) != len(native['duration'])*6
                for k in ('coef_x','coef_y','coef_z')):
            raise ValueError('invalid native quintic')
        if t > sum(native['duration']):
            return None
        piece = 0
        while piece+1 < len(native['duration']) and t > native['duration'][piece]:
            t -= native['duration'][piece]
            piece += 1
        pva = [[evaluate(native[k],5,piece,t,d) for k in ('coef_x','coef_y','coef_z')]
               for d in range(3)]
        delta = native['yaw']-self.yaw
        if delta >= math.pi:
            delta -= 2*math.pi
        if delta <= -math.pi:
            delta += 2*math.pi
        self.yaw = self.yaw+math.copysign(YAW_STEP,delta) if abs(delta)>=YAW_STEP else native['yaw']
        return pva, self.yaw

    def sample(self, now_ns):
        result = self.execute(self.latest,now_ns)
        if result is not None:
            self.previous = self.latest
            return result
        return self.execute(self.previous,now_ns)
