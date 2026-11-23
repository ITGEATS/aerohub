#!/usr/bin/env python3
"""Workstation-only ROS mission. All motion is executed by onboard AS2 -> PX4."""
import argparse,json,math,signal,sys,time
from pathlib import Path
from weststreet_ports import PORTS
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import Takeoff,GoToWaypoint,Land
from as2_msgs.msg import PlatformInfo,YawMode
from geometry_msgs.msg import PoseStamped,TwistStamped
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient

class Mission(Node):
 def __init__(self,args):
  super().__init__('weststreet_mission',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.args=args;self.out=args.output;self.out.mkdir(parents=True,exist_ok=True)
  self.pose=self.twist=self.platform=None;self.received={};self.phase='ready';self.origin=None;self.active=None;self.mission_clients=[];self.started=False
  self.result={'success':False,'topology':'pc_simulation: workstation -> onboard AS2 -> PX4 SITL -> ProjectAirSim','motion_requested':False,'collision_coverage':'unverified; not a no-collision acceptance'}
  self.stream=(self.out/'flight.jsonl').open('w');self.events=[]
  self.extension=ProjectAirSimExtensionClient(port=args.hud_port,scene_port=args.scene_port)
  for kind,topic,key in [(PoseStamped,'self_localization/pose','pose'),(TwistStamped,'self_localization/twist','twist'),(PlatformInfo,'platform/info','platform')]:
   self.create_subscription(kind,'/drone0/'+topic,lambda m,k=key:self.receive(k,m),qos_profile_sensor_data)
  self.create_subscription(Clock,'/clock',lambda m:self.received.__setitem__('clock',time.monotonic()),qos_profile_sensor_data)
  self.create_timer(.1,self.record)
 def receive(self,key,msg):setattr(self,key,msg);self.received[key]=time.monotonic()
 def position(self):return [getattr(self.pose.pose.position,k) for k in 'xyz']
 def speed(self):return math.sqrt(sum(getattr(self.twist.twist.linear,k)**2 for k in 'xyz'))
 def event(self,name,**data):
  e={'event':name,'wall_time':time.time(),'phase':self.phase,**data};self.events.append(e);print(json.dumps(e),flush=True)
  (self.out/'events.json').write_text(json.dumps(self.events,indent=2))
 def record(self):
  if not self.pose or not self.twist or not self.platform:return
  self.stream.write(json.dumps({'phase':self.phase,'sim_time':self.get_clock().now().nanoseconds/1e9,'frame':self.pose.header.frame_id,'position':self.position(),'speed_mps':self.speed(),'armed':self.platform.armed,'offboard':self.platform.offboard,'platform_state':self.platform.status.state})+'\n');self.stream.flush()
 def wait(self,predicate,timeout=20,guard=True):
  deadline=time.monotonic()+timeout
  while not predicate():
   if time.monotonic()>deadline:raise TimeoutError('Timeout in '+self.phase)
   rclpy.spin_once(self,timeout_sec=.03)
   if guard and self.origin and self.platform and self.platform.armed:
    if any(time.monotonic()-self.received.get(k,0)>1 for k in ['pose','twist','clock']):raise RuntimeError('State/clock lost')
    p=self.position()
    if math.hypot(p[0]-self.origin[0],p[1]-self.origin[1])>8 or not -.5<p[2]-self.origin[2]<4.5:raise RuntimeError('Spatial safety envelope exceeded')
 def service(self,name,value):
  c=self.create_client(SetBool,'/drone0/'+name);self.mission_clients.append(c);self.wait(c.service_is_ready)
  f=c.call_async(SetBool.Request(data=value));self.wait(f.done,guard=False)
  if not f.result().success:raise RuntimeError(name+' rejected '+str(value))
 def action(self,kind,name,goal,timeout=60):
  c=ActionClient(self,kind,'/drone0/'+name);self.mission_clients.append(c);self.wait(c.server_is_ready,guard=self.phase!='landing')
  future=c.send_goal_async(goal);self.wait(future.done,guard=self.phase!='landing')
  handle=future.result()
  if not handle.accepted:raise RuntimeError(name+' rejected goal')
  self.active=handle;future=handle.get_result_async();self.wait(future.done,timeout,guard=self.phase!='landing')
  result=future.result();self.active=None
  key={Takeoff:'takeoff_success',GoToWaypoint:'go_to_success',Land:'land_success'}[kind]
  self.event('action_result',action=name,status=result.status,result=str(result.result))
  if result.status!=4 or not getattr(result.result,key):raise RuntimeError(name+' did not succeed')
 def hold(self,seconds=2,timeout=20):
  until=None;deadline=time.monotonic()+timeout
  while until is None or time.monotonic()<until:
   self.wait(lambda:self.pose is not None and self.twist is not None,3)
   rclpy.spin_once(self,timeout_sec=.04)
   if self.speed()<.25:
    if until is None:until=time.monotonic()+seconds
   else:until=None
   if time.monotonic()>deadline:raise TimeoutError('Vehicle did not physically stop')
 def run(self):
  try:
   self.wait(lambda:all(time.monotonic()-self.received.get(k,0)<.5 for k in ['pose','twist','platform','clock']) and self.platform.connected,60)
   if self.platform.armed:raise RuntimeError('Refusing to take over an armed platform')
   self.origin=self.position();self.result.update(origin=self.origin,frame=self.pose.header.frame_id)
   for kind,name in [(Takeoff,'TakeoffBehavior'),(GoToWaypoint,'GoToBehavior'),(Land,'LandBehavior')]:
    c=ActionClient(self,kind,'/drone0/'+name);self.mission_clients.append(c);self.wait(c.server_is_ready)
   self.result['actor_initial']=self.extension.reset_actor();assert self.result['actor_initial']['state']=='paused'
   self.extension.set_title('框架短程验证');self.extension.set_title('PX4 + AS2',side='right');self.extension.set_view('Drone1','Chase')
   self.event('ground_ready',origin=self.origin,frame=self.pose.header.frame_id)
   if self.args.ground_only:self.result['success']=True;return
   q=self.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
   self.result['motion_requested']=True;self.extension.start_run('Drone1');self.started=True
   self.service('set_arming_state',True);self.wait(lambda:self.platform.armed)
   self.service('set_offboard_mode',True);self.wait(lambda:self.platform.offboard,guard=False)
   self.wait(lambda:all(time.monotonic()-self.received.get(k,0)<.5 for k in ['pose','twist','clock']),5,guard=False)
   self.phase='takeoff';self.action(Takeoff,'TakeoffBehavior',Takeoff.Goal(takeoff_height=3.,takeoff_speed=.5))
   self.hold(1)
   height=self.position()[2]-self.origin[2]
   if not 2.5<height<3.5:raise RuntimeError('Measured takeoff altitude not reached')
   self.result['takeoff_height_m']=height;self.event('takeoff_confirmed',height=height)
   self.result['actor_started']=self.extension.start_actor();self.event('actor_started_after_takeoff',state=self.result['actor_started'])
   p=self.position();target=[p[0]+5*math.cos(yaw),p[1]+5*math.sin(yaw),p[2]];self.result['target']=target
   goal=GoToWaypoint.Goal();goal.target_pose.header.frame_id=self.pose.header.frame_id;goal.target_pose.header.stamp=self.get_clock().now().to_msg()
   goal.target_pose.point.x,goal.target_pose.point.y,goal.target_pose.point.z=target;goal.max_speed=1.;goal.yaw.mode=YawMode.KEEP_YAW
   self.phase='forward';self.action(GoToWaypoint,'GoToBehavior',goal)
   self.phase='stopping';self.hold(2)
   error=math.dist(self.position(),target);self.result['position_error_m']=error;self.result['stop_speed_mps']=self.speed()
   if error>.75:raise RuntimeError('Actual endpoint error exceeded .75m')
   self.result['route_and_stop_passed']=True;self.event('forward_and_stop_confirmed',error=error,speed=self.speed())
  except BaseException as exc:
   self.result['error']=repr(exc);self.event('mission_error',error=repr(exc))
  finally:
   if self.platform and self.platform.armed:
    try:
     self.phase='landing'
     if self.active:
      pending=self.active.cancel_goal_async();self.wait(pending.done,10,guard=False);self.active=None
     self.action(Land,'LandBehavior',Land.Goal(land_speed=.5),90)
     self.result['land_success']=True;self.wait(lambda:not self.platform.armed,12,guard=False)
    except BaseException as exc:self.result['recovery_error']=repr(exc)
   self.result['disarmed']=bool(self.platform and not self.platform.armed)
   if self.result['disarmed'] and self.result['motion_requested']:
    try:self.service('set_offboard_mode',False)
    except Exception as exc:self.result['release_error']=repr(exc)
   try:self.result['actor_final']=self.extension.stop_actor()
   except Exception as exc:self.result['actor_stop_error']=repr(exc)
   if self.started:
    try:self.extension.finish_run('Drone1')
    except Exception as exc:self.result['hud_error']=repr(exc)
   if not self.args.ground_only:self.result['success']=bool(self.result.get('route_and_stop_passed') and self.result.get('land_success') and self.result['disarmed'] and not self.result.get('error') and not self.result.get('recovery_error'))
   self.record();self.stream.close();(self.out/'result.json').write_text(json.dumps(self.result,indent=2));print(json.dumps(self.result),flush=True)

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--ground-only',action='store_true');p.add_argument('--hud-port',type=int,default=PORTS['hud']);p.add_argument('--scene-port',type=int,default=PORTS['scene']);a=p.parse_args()
 rclpy.init();node=Mission(a)
 def stop(*_):raise RuntimeError('Mission canceled; native Land recovery requested')
 signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
 try:node.run()
 finally:node.destroy_node();rclpy.try_shutdown()
 raise SystemExit(0 if node.result['success'] else 1)
