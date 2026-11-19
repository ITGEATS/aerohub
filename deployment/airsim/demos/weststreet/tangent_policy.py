"""Experimental trailing-position reference from Tracking p/v, not a trajectory planner."""
import math


class TangentReference:
    def __init__(self, offset, initial, distance=8., lead_seconds=.35):
        self.heading=math.atan2(-offset[1],-offset[0])
        self.point=tuple(initial)
        if not math.isfinite(lead_seconds) or not 0<=lead_seconds<=1.5:
            raise ValueError("lead_seconds must be within [0,1.5]")
        self.lead_seconds=lead_seconds
        self.distance=distance

    def update(self, position, velocity, dt):
        if not 0<dt<=1 or not all(math.isfinite(x)for x in (*position,*velocity)):
            raise ValueError('Invalid tangent reference sample')
        speed=math.hypot(*velocity)
        heading_step=0.
        if speed>=.8:
            desired=math.atan2(velocity[1],velocity[0])
            error=math.atan2(math.sin(desired-self.heading),math.cos(desired-self.heading))
            heading_step=max(-.8*dt,min(.8*dt,error*(1-math.exp(-dt/.25))))
            self.heading+=heading_step
        # Small lead accounts for reference age; does not provide velocity feedforward.
        raw=tuple(position[k]+self.lead_seconds*velocity[k]-self.distance*(math.cos(self.heading)if k==0 else math.sin(self.heading)) for k in range(2))
        delta=math.dist(raw,self.point)
        fraction=min(1.,7.*dt/max(delta,1e-9))
        self.point=tuple(a+fraction*(b-a)for a,b in zip(self.point,raw))
        return self.point,dict(speed=speed,heading=self.heading,heading_step=heading_step,
            requested_point=raw,requested_reference_speed=delta/dt,reference_limited=fraction<1.,
            lead_seconds=self.lead_seconds,max_heading_rate=.8,max_reference_speed=7.)
