"""Offline, metre-based circular fillets and distance-synchronised speed profile."""
import math


def rounded_path(points, radius=4.0, spacing=0.2, corners=None, *, max_trim_fraction=.45):
    """Round selected interior corners; reject insufficient room, never shrink radius."""
    if radius <= 0 or spacing <= 0:
        raise ValueError('radius and spacing must be positive')
    out = [tuple(points[0])]
    def line(end):
        start = out[-1]
        n = max(1, math.ceil(math.dist(start, end) / spacing))
        out.extend(tuple(a + (b-a)*i/n for a,b in zip(start,end)) for i in range(1,n+1))
    for i in range(1, len(points)-1):
        a,b,c = points[i-1:i+2]
        if corners is not None and i not in corners:
            line(b)
            continue
        incoming = tuple((b[k]-a[k])/math.dist(a,b) for k in range(2))
        outgoing = tuple((c[k]-b[k])/math.dist(b,c) for k in range(2))
        angle = math.acos(max(-1,min(1,sum(x*y for x,y in zip(incoming,outgoing)))))
        if angle < 1e-6:
            line(b)
            continue
        trim = radius * math.tan(angle/2)
        if trim > min(math.dist(a,b), math.dist(b,c))*max_trim_fraction:
            raise ValueError('Not enough room for requested radius at corner %d' % i)
        entry = tuple(b[k]-trim*incoming[k] for k in range(2))
        exit = tuple(b[k]+trim*outgoing[k] for k in range(2))
        sign = 1 if incoming[0]*outgoing[1]-incoming[1]*outgoing[0]>0 else -1
        center = (entry[0]-sign*radius*incoming[1], entry[1]+sign*radius*incoming[0])
        start_angle = math.atan2(entry[1]-center[1],entry[0]-center[0])
        line(entry)
        n = max(1, math.ceil(radius*angle/spacing))
        out.extend((center[0]+radius*math.cos(start_angle+sign*angle*j/n),
                    center[1]+radius*math.sin(start_angle+sign*angle*j/n)) for j in range(1,n+1))
        out[-1] = exit
    line(points[-1])
    return out


def profile(points, straight=5., turn=2.75, acceleration=1.25):
    """Curvature-triggered slowdown with cyclic forward/backward acceleration bounds.

    Inputs must be a closed loop. Linear segment speeds give a continuous speed
    profile in arc length. Animation phase is arc length / straight speed.
    """
    if not (0 < turn <= straight and acceleration > 0) or points[0] != points[-1]:
        raise ValueError('positive speeds/acceleration and closed loop required')
    p=points[:-1]
    ds=[math.dist(p[i],p[(i+1)%len(p)]) for i in range(len(p))]
    if min(ds)<=1e-9:
        raise ValueError('duplicate adjacent points')
    speed=[]
    for i,b in enumerate(p):
        a,c=p[i-1],p[(i+1)%len(p)]
        x=(b[0]-a[0],b[1]-a[1]);y=(c[0]-b[0],c[1]-b[1])
        angle=abs(math.atan2(x[0]*y[1]-x[1]*y[0],sum(v*w for v,w in zip(x,y))))
        curvature=angle/((ds[i-1]+ds[i])/2)
        speed.append(turn if curvature > .02 else straight)
    # Propagate bounds around the seam as well as both sides of every bend.
    for _ in range(len(p)):
        changed=False
        for indices in (range(len(p)),range(len(p)-1,-1,-1)):
            for i in indices:
                j=(i+1)%len(p)
                vi=min(speed[i],math.sqrt(speed[j]**2+2*acceleration*ds[i]))
                vj=min(speed[j],math.sqrt(vi**2+2*acceleration*ds[i]))
                changed |= vi < speed[i]-1e-10 or vj < speed[j]-1e-10
                speed[i],speed[j]=vi,vj
        if not changed:break
    speed.append(speed[0]);times=[0.];distance=[0.]
    for i,d in enumerate(ds):
        times.append(times[-1]+2*d/(speed[i]+speed[i+1]));distance.append(distance[-1]+d)
    return dict(points_m=points,speeds_mps=speed,times_s=times,distance_m=distance,
                animation_phase_s=[d/straight for d in distance],duration_s=times[-1])


def sample(plan, t):
    """Position and travelled distance with constant acceleration per segment."""
    import bisect
    t=max(0.,min(t,plan['duration_s']))
    if t == plan['duration_s']:
        return tuple(plan['points_m'][-1]), plan['distance_m'][-1]
    i=min(len(plan['times_s'])-2,bisect.bisect_right(plan['times_s'],t)-1)
    dt=t-plan['times_s'][i]
    duration=plan['times_s'][i+1]-plan['times_s'][i]
    v=plan['speeds_mps'][i]
    a=(plan['speeds_mps'][i+1]-v)/duration
    d=v*dt+.5*a*dt*dt
    length=plan['distance_m'][i+1]-plan['distance_m'][i]
    fraction=d/length
    pos=tuple(x+(y-x)*fraction for x,y in zip(plan['points_m'][i],plan['points_m'][i+1]))
    return pos,plan['distance_m'][i]+d


def candidate_from_baseline(original):
    """Join immediately after the third fillet, preserving the obstacle detour.

    The reviewed outgoing straight is not exactly east/west: intersect its
    tangent with x=122 rather than shortcutting all the way to plaza entry.
    """
    join=min(range(len(original)),key=lambda i:math.dist(original[i],(112.,-9.25)))
    a,b=original[join:join+2]
    if math.dist(a,(112.,-9.25))>1 or abs(b[0]-a[0])<.1:
        raise ValueError('Unexpected baseline outgoing tangent')
    y=a[1]+(122-a[0])*(b[1]-a[1])/(b[0]-a[0])
    first=rounded_path([(4,-40),(4,64),(122,64),(122,y),a],4.,.2)
    points=first+original[join+1:]
    points[-1]=points[0]
    return points


def shifted_first_junction(points, shift_m=2.):
    """Right of initial +UE-Y heading is -UE-X; preserve start and obstacle suffix.

    Ramp over 40m before the junction, keep a rigid translation across the 4m
    fillet, then merge along the outgoing straight. Keeping the initial point
    avoids changing the Mission's initially derived world-frame follow offset.
    """
    if not math.isfinite(shift_m) or not 0<shift_m<=3.:
        raise ValueError('shift_m must be within (0,3]')
    distance=0.;result=[]
    smooth=lambda t: t*t*(3-2*t)
    for i,p in enumerate(points):
        if i:distance+=math.dist(points[i-1],p)
        if distance<=20:weight=0.
        elif distance<60:weight=smooth((distance-20)/40)
        elif distance<=130:weight=1.
        elif distance<170:weight=1-smooth((distance-130)/40)
        else:weight=0.
        result.append((p[0]-shift_m*weight,p[1]))
    return result


def delayed_first_junction(points, delay_m=4.):
    """Extend incoming +Y straight, translate first fillet, then merge back."""
    if not math.isfinite(delay_m) or not 0 < delay_m <= 6:
        raise ValueError('delay_m must be within (0,6]')
    distance=0.; result=[]
    smooth=lambda t:t*t*(3-2*t)
    for i,p in enumerate(points):
        if i:distance+=math.dist(points[i-1],p)
        if distance<=40:weight=0.
        elif distance<80:weight=smooth((distance-40)/40)
        elif distance<=130:weight=1.
        elif distance<170:weight=1-smooth((distance-130)/40)
        else:weight=0.
        result.append((p[0],p[1]+delay_m*weight))
    return result


def close_return_lane(points, lane_x=13.5, turn_y=-49.):
    """Replace the reversing tail with a rounded return to the start tangent."""
    start = points[0]
    join = next(i for i in range(len(points)-2, 0, -1)
                if math.dist(points[i], (18., -12.)) < 1e-6)
    # Start the fillet on the existing -X straight, and approach the
    # original start along +Y so the loop seam lies on a straight.
    anchor = points[join]
    lane_x = float(lane_x)
    turn_y = float(turn_y)
    if lane_x >= anchor[0] or lane_x - start[0] < 9 or turn_y > start[1] - 9:
        raise ValueError('Return lane does not leave room for 4 m bends')
    # The first segment starts on an already validated straight. Its 4 m
    # trim fits in 4.5 m, and the 9.5 m cross street fits both 4 m trims.
    return_lane = rounded_path(
        [anchor, (lane_x, anchor[1]), (lane_x, turn_y),
         (start[0], turn_y), start], radius=4., spacing=.2, max_trim_fraction=.99)
    return points[:join] + return_lane
