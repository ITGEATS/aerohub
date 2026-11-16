"""Native AS2 flight supervision for a private PX4 SITL only; no pursuit/reference publisher."""
import json
import math
from pathlib import Path
import threading
import time

from action_msgs.msg import GoalStatus
from as2_msgs.action import Takeoff, Land
from as2_msgs.msg import ControllerInfo, PlatformInfo, PlatformStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import VehicleStatus, VehicleControlMode, VehicleLandDetected, VehicleCommand, TrajectorySetpoint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
from rosgraph_msgs.msg import Clock


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class Flight(Node):
    def __init__(self, output):
        super().__init__('mountains_hover_supervisor')
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.latest = {}; self.samples = []; self.events = []
        self.lock = threading.RLock()
        self.active_handle = self.active_result = None
        self.task_clients = []
        self.phase = 'ground'
        self.last_clock = None
        self.clock_regressed = False
        self.clock_received = 0.
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.ground = None
        self.last_health = None
        self.arming_attempted = False
        self.result = dict(takeoff_success=False, hover_stable=False,
                           land_success=False, disarmed_ground=False)
        for key, topic, kind in (
            ('pose', 'self_localization/pose', PoseStamped),
            ('twist', 'self_localization/twist', TwistStamped),
            ('platform', 'platform/info', PlatformInfo),
            ('controller', 'controller/info', ControllerInfo),
            ('status', 'fmu/out/vehicle_status_v4', VehicleStatus),
            ('mode', 'fmu/out/vehicle_control_mode', VehicleControlMode),
            ('land', 'fmu/out/vehicle_land_detected', VehicleLandDetected),
            ('land', 'fmu/out/vehicle_land_detected_v1', VehicleLandDetected)):
            self.create_subscription(kind, topic, lambda m, k=key: self.receive(k, m), qos_profile_sensor_data)
        self.create_subscription(VehicleCommand, 'fmu/in/vehicle_command',
            lambda m: self.event('px4_command', command=m.command, p1=m.param1, p2=m.param2), qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoint, 'fmu/in/trajectory_setpoint',
            lambda m: self.event('px4_setpoint', position=[float(x) for x in m.position],
                                 velocity=[float(x) for x in m.velocity]), qos_profile_sensor_data)
        self.create_timer(.1, self.sample)

    def on_clock(self, msg):
        value=seconds(msg.clock)
        with self.lock:
            if self.last_clock is not None and value < self.last_clock:
                self.clock_regressed=True
                self.event('clock_regression', previous=self.last_clock, current=value)
            self.last_clock=value
            self.clock_received=time.monotonic()

    def event(self, event, **fields):
        with self.lock:
            self.events.append(dict(event=event, phase=self.phase, wall=time.time(),
                                    monotonic=time.monotonic(), **fields))

    def receive(self, key, msg):
        with self.lock:
            self.latest[key] = (msg, time.monotonic())

    def snapshot(self):
        with self.lock:
            items = dict(self.latest)
        now = self.get_clock().now().nanoseconds / 1e9
        wall = time.monotonic()
        result = dict(phase=self.phase, wall=time.time(), monotonic=wall, ros_time=now)
        for key, (msg, received) in items.items():
            rec = dict(received_monotonic=received, receive_age=wall-received)
            if hasattr(msg, 'header'):
                rec.update(stamp=seconds(msg.header.stamp), frame=msg.header.frame_id,
                           age=now-seconds(msg.header.stamp))
            if key == 'pose':
                p=msg.pose.position; q=msg.pose.orientation
                rec.update(p=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w])
            elif key == 'twist':
                v=msg.twist.linear; rec['v']=[v.x,v.y,v.z]
            elif key == 'platform':
                rec.update(armed=msg.armed,offboard=msg.offboard,connected=msg.connected,state=msg.status.state)
            elif key == 'status':rec.update(preflight=msg.pre_flight_checks_pass,arming_state=msg.arming_state,system_id=msg.system_id,failsafe=msg.failsafe)
            elif key == 'mode':rec.update(armed=msg.flag_armed,offboard=msg.flag_control_offboard_enabled)
            elif key == 'land':rec.update(landed=msg.landed,ground_contact=msg.ground_contact)
            elif key == 'controller':rec.update(input_mode=msg.input_control_mode.control_mode,output_mode=msg.output_control_mode.control_mode)
            result[key]=rec
        return result

    def sample(self):
        sample=self.snapshot()
        with self.lock:self.samples.append(sample)
        (self.output/'flight-live.json').write_text(json.dumps(sample)+'\n')

    def wait(self, predicate, timeout, label):
        end=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>=end:
                self.event('timeout',label=label,snapshot=self.snapshot())
                raise TimeoutError(label)
            time.sleep(.02)

    def fresh(self, s, key, age=.5):
        v=s.get(key,{})
        return (0 <= v.get('receive_age',float('inf')) <= age and
                ('stamp' not in v or 0 <= v.get('age',float('inf')) <= age))

    def healthy(self, airborne=False):
        s=self.snapshot()
        okay=(not self.clock_regressed and 0 <= time.monotonic()-self.clock_received <= 1.
              and all(self.fresh(s,k) for k in ('pose','twist'))
              and all(self.fresh(s,k,1.5) for k in ('platform','controller','status','mode','land'))
              and s.get('pose',{}).get('frame')=='earth'
              and s.get('platform',{}).get('connected',False))
        if airborne:
            okay=(okay and s['platform']['armed'] and s['platform']['offboard']
                  and s['mode']['armed'] and s['mode']['offboard']
                  and s['platform']['state']==PlatformStatus.FLYING and not s['land']['landed'])
        self.last_health=s
        return okay

    def preflight(self):
        self.wait(lambda:self.healthy() and not self.snapshot()['platform']['armed']
                  and self.snapshot()['land']['landed'],45,'fresh disarmed ground state/control stack')
        expected=['/clock','/drone0/self_localization/pose','/drone0/self_localization/twist']
        publishers={t:self.count_publishers(t) for t in expected}
        self.event('publisher_authority',publishers=publishers)
        if any(v!=1 for v in publishers.values()):raise RuntimeError('competing/missing state or clock publisher')
        stable=None;end=time.monotonic()+45
        while time.monotonic()<end:
            s=self.snapshot()
            if self.healthy() and s['status']['preflight'] and s['status']['system_id']==81 and not s['status']['failsafe']:
                if stable is None:stable=time.monotonic()
                if time.monotonic()-stable>=1.:break
            else:stable=None
            time.sleep(.05)
        else:raise RuntimeError('PX4 preflight never passed for 1s; no arming')
        for kind,name in ((Takeoff,'TakeoffBehavior'),(Land,'LandBehavior')):
            client=ActionClient(self,kind,name);self.task_clients.append(client)
            self.wait(client.server_is_ready,5,name+' ready')
        self.ground=self.snapshot()['pose']['p']
        self.event('preflight_passed',snapshot=self.snapshot())

    def service(self,name,value,timeout=8):
        client=self.create_client(SetBool,name);self.task_clients.append(client)
        self.wait(client.service_is_ready,timeout,name+' ready')
        future=client.call_async(SetBool.Request(data=value));self.wait(future.done,timeout,name+' response')
        response=future.result();self.event('service',name=name,value=value,response=str(response))
        if not response.success:raise RuntimeError(name+' rejected')

    def action(self,kind,name,goal,field,timeout=40):
        client=ActionClient(self,kind,name);self.task_clients.append(client)
        self.wait(client.server_is_ready,5,name+' ready')
        future=client.send_goal_async(goal);self.wait(future.done,5,name+' accept')
        handle=future.result()
        self.event('action_accept',name=name,accepted=handle.accepted,goal=str(goal))
        if not handle.accepted:raise RuntimeError(name+' rejected')
        result=handle.get_result_async();self.active_handle=handle;self.active_result=result
        self.wait(result.done,timeout,name+' result')
        outcome=result.result()
        self.event('action_result',name=name,status=outcome.status,result=str(outcome.result))
        self.active_handle=self.active_result=None
        if outcome.status!=GoalStatus.STATUS_SUCCEEDED or not getattr(outcome.result,field):raise RuntimeError(name+' failed')
        self.result[field]=True

    def takeoff(self):
        self.phase='arming';self.arming_attempted=True
        self.service('set_arming_state',True)
        self.wait(lambda:self.snapshot().get('platform',{}).get('armed'),5,'armed feedback')
        self.service('set_offboard_mode',True)
        self.wait(lambda:self.snapshot().get('mode',{}).get('offboard'),5,'PX4 offboard feedback')
        self.phase='takeoff'
        self.action(Takeoff,'TakeoffBehavior',Takeoff.Goal(takeoff_height=3.,takeoff_speed=.5),'takeoff_success')
        self.phase='hover'
        self.wait_stable()

    def hover_check(self):
        s=self.snapshot()
        if not self.healthy(airborne=True) or self.ground is None:return False
        p=s['pose']['p'];v=s['twist']['v']
        return (all(math.isfinite(x) for x in p+v) and abs(p[2]-self.ground[2]-3.)<=.3
                and math.dist(p[:2],self.ground[:2])<=.5 and math.sqrt(sum(x*x for x in v))<=.2)

    def wait_stable(self):
        start=None;end=time.monotonic()+15
        while time.monotonic()<end:
            if self.hover_check():
                if start is None:start=time.monotonic()
                if time.monotonic()-start>=2.:
                    self.result['hover_stable']=True;self.event('hover_stable',snapshot=self.snapshot());return
            else:start=None
            time.sleep(.05)
        raise RuntimeError('3m hover did not meet 2s height/speed/drift/freshness window')

    def cancel_active(self):
        if self.active_handle is not None and self.active_result is not None and not self.active_result.done():
            future=self.active_handle.cancel_goal_async();self.wait(future.done,5,'active takeoff cancel')
            self.wait(self.active_result.done,5,'active takeoff drained')
            self.event('action_drained_before_land',status=self.active_result.result().status)
        self.active_handle=self.active_result=None

    def recover(self):
        self.phase='landing'
        errors=[]
        self.wait(lambda:'platform' in self.snapshot() and 'land' in self.snapshot() and 'mode' in self.snapshot(),10,'recovery state')
        s=self.snapshot()
        if s['platform']['armed'] or s['mode']['armed'] or not s['land']['landed']:
            try:self.cancel_active()
            except Exception as exc:errors.append('cancel: '+repr(exc))
            # Land is independent of a failed cancel; never skip it in that exception path.
            try:self.action(Land,'LandBehavior',Land.Goal(land_speed=.4),'land_success',45)
            except Exception as exc:errors.append('land: '+repr(exc))
            try:
                self.wait(lambda:self.grounded(),12,'native land disarmed/ground feedback')
            except Exception as exc:
                # Only an actual landed vehicle may receive this disarm fallback.
                s=self.snapshot()
                if self.fresh(s,'land') and s['land']['landed'] and s.get('platform',{}).get('armed'):
                    try:self.service('set_arming_state',False)
                    except Exception as extra:errors.append('landed disarm: '+repr(extra))
                else:errors.append('ground confirmation: '+repr(exc))
        self.wait(self.grounded,10,'final disarmed landed')
        self.result['disarmed_ground']=True
        self.result['recovery_errors']=errors
        self.event('disarmed_ground',snapshot=self.snapshot())
        self.save()

    def grounded(self):
        s=self.snapshot()
        return (all(self.fresh(s,k,1.5) for k in ('platform','land','mode')) and
                not s['platform']['armed'] and not s['mode']['armed'] and s['land']['landed'])

    def save(self):
        with self.lock:
            for name,data in [('flight-result',self.result),('flight-samples',self.samples),('flight-events',self.events)]:
                (self.output/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
