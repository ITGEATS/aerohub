"""Experimental short-horizon candidate scoring; not an obstacle/visibility planner."""
import math


def select_observation_point(own, velocity, yaw, target, target_velocity, *, distance=8., previous=None):
    horizon=1.5
    base=math.atan2(own[1]-target[1],own[0]-target[0])
    candidates=[]
    for angle in (0,-.35,.35,-.7,.7,-1.05,1.05):
        # Predict endpoint; preserve observation side unless rollout favours a change.
        goal=tuple(target[k]+target_velocity[k]*horizon+distance*(math.cos(base+angle) if k==0 else math.sin(base+angle)) for k in range(2))
        p=list(own);v=list(velocity);heading=yaw;minimum=math.dist(own,target);max_rate=0.;cost=0.
        for i in range(1,16):
            dt=.1;t=i*dt
            desired=[max(-7.,min(7.,(goal[k]-p[k])*1.2)) for k in range(2)]
            a=[(desired[k]-v[k])*2 for k in range(2)];norm=math.hypot(*a)
            if norm>2.:a=[x*2/norm for x in a]
            for k in range(2):v[k]+=a[k]*dt;p[k]+=v[k]*dt
            z=[target[k]+target_velocity[k]*t for k in range(2)]
            gap=math.dist(p,z);minimum=min(minimum,gap)
            bearing=math.atan2(z[1]-p[1],z[0]-p[0]);error=math.atan2(math.sin(bearing-heading),math.cos(bearing-heading))
            max_rate=max(max_rate,abs(error)/dt)
            heading+=max(-1.5*dt,min(1.5*dt,error))
            cost+=20*max(0.,6.-gap)**2+.2*(gap-distance)**2+8*max(0.,abs(error)-.45)**2
        cost+=.05*math.dist(goal,previous or goal)**2+.5*angle**2
        candidates.append(dict(goal=goal,cost=cost,predicted_min_distance=minimum,max_requested_yaw_rate=max_rate,angle_offset=angle))
    best=min(candidates,key=lambda x:x['cost'])
    return best['goal'],dict(best,candidates=len(candidates),horizon=horizon,model='bounded-acceleration surrogate, not SUPER trajectory')
