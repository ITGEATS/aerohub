"""Native zero-length spring-arm geometry; pure math in ROS FLU convention."""
import math
from sensor_contract import extrinsic
from uav_as2_sensor_adapter.conversion import multiply


def normalize(q):
    if len(q)!=4 or not all(math.isfinite(x) for x in q):
        raise ValueError('invalid quaternion')
    norm=math.sqrt(sum(x*x for x in q))
    if norm<1e-9:raise ValueError('zero quaternion')
    return tuple(x/norm for x in q)


def interpolate(a,b,t):
    a,b=normalize(a),normalize(b)
    dot=sum(x*y for x,y in zip(a,b))
    if dot<0:b=tuple(-x for x in b);dot=-dot
    dot=min(1.,max(-1.,dot))
    if dot>.9995:return normalize(tuple(x+t*(y-x) for x,y in zip(a,b)))
    angle=math.acos(dot)
    return tuple((math.sin((1-t)*angle)*x+math.sin(t*angle)*y)/math.sin(angle) for x,y in zip(a,b))


def stabilized_transform(body_quaternion, origin):
    q=normalize(body_quaternion)
    x,y,z,w=q
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    inverse=(-x,-y,-z,w)
    compensation=multiply(inverse,(0.,0.,math.sin(yaw/2),math.cos(yaw/2)))
    xyz,optical=extrinsic(origin,optical=True)
    conjugate=tuple(-x for x in compensation[:3])+(compensation[3],)
    moved=multiply(multiply(compensation,tuple(xyz)+(0.,)),conjugate)
    return moved[:3],multiply(compensation,optical)
