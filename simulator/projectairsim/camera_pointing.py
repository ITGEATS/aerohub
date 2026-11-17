"""Platform binding: visual Tracking state -> native gimbal, no Actor truth."""
import copy
import json
import math
import time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from projectairsim_ros2_cpp.srv import RawRequest
from uav_target_tracking_interfaces.msg import TargetState


def pointing_angles(own, body_yaw, target):
    dx,dy,dz=(b-a for a,b in zip(own,target))
    if not all(math.isfinite(x)for x in(dx,dy,dz,body_yaw)) or math.hypot(dx,dy)<.5:
        raise ValueError('invalid pointing geometry')
    yaw=-(math.atan2(dy,dx)-body_yaw)
    yaw=math.atan2(math.sin(yaw),math.cos(yaw))
    return max(-35.,min(20.,math.degrees(math.atan2(dz,math.hypot(dx,dy))))),max(-100.,min(100.,math.degrees(yaw)))


class CameraPointing:
    def __init__(self,node,stream):
        self.node,self.stream=node,stream
        self.target=None;self.future=None;self.phase=None;self.before=None
        self.changed=0.;self.error='';self.history=[(None,0,copy.deepcopy(stream['gimbal_origin']))]
        self.current=copy.deepcopy(stream['gimbal_origin']);self.next=None
        self.buffer=Buffer(node=node);self.listener=TransformListener(self.buffer,node,qos=qos_profile_sensor_data)
        self.client=node.create_client(RawRequest,'/projectairsim/request')
        self.subscription=node.create_subscription(TargetState,stream['pointing_target'],self.receive,10)
        self.publisher=node.create_publisher(String,stream['parent'].split('/')[0]+'/gimbal/status',10)
        self.timer=node.create_timer(.02,self.tick)
        self.world='/Sim/'+stream['source'].split('/')[2]
        self.camera=self.world+'/robots/'+stream['source'].split('/')[4]+'/sensors/'+stream['source'].split('/')[6]

    def receive(self,state):
        self.target=(state,time.monotonic()) if state.valid else None

    def request(self,method,params):
        self.sent=time.monotonic()
        self.future=self.client.call_async(RawRequest.Request(method=method,json_parameters=json.dumps(params)))

    def origin_at(self,stamp):
        if self.error:return None
        if self.before is not None and stamp>=self.before:return None
        for before,after,origin in reversed(self.history):
            if stamp>=after:return None if before is not None and stamp<=after else origin
            if before is not None and stamp>=before:return None
        return None

    def tick(self):
        now=self.node.get_clock().now().nanoseconds
        self.publisher.publish(String(data=json.dumps(dict(phase=self.phase,error=self.error,
            applied_pitch=self.current['Pitch'],applied_yaw=self.current['Yaw'],confirmed_changes=len(self.history)-1))))
        if self.error:return
        try:
            if self.future is not None:
                if not self.future.done():
                    if time.monotonic()-self.sent>1.:raise RuntimeError('gimbal RPC outcome unconfirmed')
                    return
                response=self.future.result();self.future=None
                if not response.success:raise RuntimeError(response.status)
                value=json.loads(response.result_json)
                if self.phase=='before':
                    self.before=int(value)
                    p,y=map(math.radians,(self.next['Pitch'],self.next['Yaw']))
                    q=dict(w=math.cos(y/2)*math.cos(p/2),x=-math.sin(y/2)*math.sin(p/2),
                           y=math.cos(y/2)*math.sin(p/2),z=math.sin(y/2)*math.cos(p/2))
                    pose=dict(frame_id='DEFAULT_FRAME',translation=dict(x=self.next['X'],y=self.next['Y'],z=self.next['Z']),rotation=q)
                    self.phase='set'
                    self.request(self.camera+'/SetPose',dict(pose=pose,wait_for_pose_update=True));return
                if self.phase=='set':
                    if value is not True:raise RuntimeError('camera pose rejected')
                    self.phase='after';self.request(self.world+'/GetSimTime',{});return
                after=int(value)
                if after<self.before:raise RuntimeError('clock reset during gimbal update')
                self.current=self.next
                self.history.append((self.before,after,copy.deepcopy(self.current)))
                self.history=self.history[-256:]
                self.before=None;self.phase=None;self.changed=time.monotonic()
            if self.target is None or time.monotonic()-self.changed<.2 or not self.client.service_is_ready():return
            state,received=self.target
            obs=state.observation_stamp.sec*10**9+state.observation_stamp.nanosec
            valid_until=state.valid_until.sec*10**9+state.valid_until.nanosec
            if time.monotonic()-received>.2 or not 0<=now-obs<=500_000_000 or now>valid_until:return
            tf=self.buffer.lookup_transform(state.header.frame_id,self.stream['parent'],Time.from_msg(state.header.stamp))
            p,q=tf.transform.translation,tf.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            # Native stable spring arm rotates the mount by body yaw only.
            x,y,z=self.current['X'],self.current['Y'],self.current['Z']
            own=(p.x+math.cos(yaw)*x+math.sin(yaw)*y,p.y+math.sin(yaw)*x-math.cos(yaw)*y,p.z-z)
            pitch,relative_yaw=pointing_angles(own,yaw,(state.position.x,state.position.y,state.position.z))
            dp,dy=pitch-self.current['Pitch'],relative_yaw-self.current['Yaw']
            if max(abs(dp),abs(dy))<2.:return
            self.next=copy.deepcopy(self.current)
            self.next['Pitch']+=max(-9.,min(9.,dp));self.next['Yaw']+=max(-12.,min(12.,dy))
            self.phase='before';self.request(self.world+'/GetSimTime',{})
        except Exception as error:
            # TF may arrive after target feedback; retry within freshness budget.
            if self.phase is None and self.before is None:
                return
            self.error=str(error)
