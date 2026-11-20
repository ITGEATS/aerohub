"""Read-only real ROS readiness; no command publisher, trigger or goal submission."""
import json,time
from pathlib import Path
from collections import Counter
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,PointCloud2
from px4_msgs.msg import VehicleStatus,VehicleLandDetected,VehicleCommand
from as2_msgs.msg import PlatformInfo,PlatformStatus
from as2_msgs.action import Takeoff,Land
from exploration_manager.msg import DroneState
from plan_env.msg import ChunkStamps

OUT=Path('/result');F=json.loads(Path('/config/fleet.json').read_text());C=json.loads(Path('/config/flight.json').read_text())
class Probe(Node):
    def __init__(self):
        super().__init__('racer_complete_ground_readiness',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.expected_budget = json.loads(Path('/evidence/local.json').read_text()).get('completion_wall_budget_s')
        self.rows={};self.rx={};self.counts=Counter();self.armed=False;self.commands=[];self.peer_ids=set();self.stamp_ids=set();self.actions={};self.parameters={};self.param_clients={};self.futures={}
        self.log=(OUT/'samples.jsonl').open('x',buffering=1)
        self.names=[f'sdf_map.box_{edge}_{axis}' for axis in 'xyz' for edge in ('min','max')]+['exploration.drone_num','exploration.drone_id','external_execution.enabled']
        for v in F['vehicles']:
            ns=v['namespace'];p='/racer_multi/'+ns
            for key,typ,topic in [('inputs',String,p+'/input_state'),('native',String,p+'/native_state'),('adapter',String,p+'/adapter/state'),('odom',Odometry,'/'+ns+'/navigation/odom'),('registered',Odometry,p+'/odometry'),('depth',Image,p+'/depth'),('map',PointCloud2,p+'/occupancy'),('platform',PlatformInfo,'/'+ns+'/platform/info'),('px4',VehicleStatus,'/'+ns+'/fmu/out/vehicle_status_v4'),('land',VehicleLandDetected,'/'+ns+'/fmu/out/vehicle_land_detected')]:
                self.create_subscription(typ,topic,lambda m,n=ns,k=key:self.receive(n,k,m),10 if typ==String else qos_profile_sensor_data)
            self.create_subscription(VehicleCommand,'/'+ns+'/fmu/in/vehicle_command',lambda m,n=ns:self.commands.append({'vehicle':n,'message':message_to_ordereddict(m)}),qos_profile_sensor_data)
            self.actions[ns]=[ActionClient(self,typ,'/'+ns+'/'+name) for typ,name in [(Takeoff,'TakeoffBehavior'),(Land,'LandBehavior')]]
            self.param_clients[ns]=self.create_client(GetParameters,'/racer_'+ns+'/get_parameters')
        self.create_subscription(String,'/racer_landing/observation',lambda m:self.receive('fleet','truth',m),10)
        self.create_subscription(DroneState,'/swarm_expl/drone_state',lambda m:self.peer_ids.add(m.drone_id),100)
        self.create_subscription(ChunkStamps,'/multi_map_manager/chunk_stamps',lambda m:self.stamp_ids.add(m.from_drone_id),100)
        self.create_timer(1.,self.sample)
    def receive(self,ns,key,m):
        self.rx[ns,key]=time.monotonic();self.counts[ns+'/'+key]+=1
        if key=='depth':value={'stamp':message_to_ordereddict(m.header),'width':m.width,'height':m.height,'encoding':m.encoding,'bytes':len(m.data)}
        elif key=='map':value={'stamp':message_to_ordereddict(m.header),'points':m.width*m.height}
        else:value=json.loads(m.data) if isinstance(m,String) else message_to_ordereddict(m)
        self.rows.setdefault(ns,{})[key]=value
        if key=='px4' and m.arming_state!=1:self.armed=True
        if key=='platform' and m.armed:self.armed=True
    def sample(self):
        self.log.write(json.dumps({'wall_ns':time.time_ns(),'sim_ns':self.get_clock().now().nanoseconds,'rows':self.rows})+'\n')
    def check(self):
        checks={};now=time.monotonic();sim=self.get_clock().now().nanoseconds
        checks['no_arming_observed']=not self.armed
        checks['no_motion_command_observed']=not self.commands
        checks['truth_fresh']=now-self.rx.get(('fleet','truth'),0)<1.25 and not self.rows.get('fleet',{}).get('truth',{}).get('error')
        if 'completion_wall_budget_s' in C:
            checks['completion_budget_matches_case'] = C['completion_wall_budget_s'] == self.expected_budget
        ids={v['racer_id'] for v in F['vehicles']}
        checks['both_swarm_states_received']=ids<=self.peer_ids
        checks['both_map_stamp_senders_received']=ids<=self.stamp_ids
        for v in F['vehicles']:
            ns=v['namespace'];d=self.rows.get(ns,{});p=d.get('px4',{});plat=d.get('platform',{});i=d.get('inputs',{});n=d.get('native',{});a=d.get('adapter',{})
            checks[ns+'/telemetry_fresh']=all(now-self.rx.get((ns,k),0)<1.25 for k in ('inputs','native','adapter','odom','registered','platform','px4','land'))
            checks[ns+'/preflight']=p.get('arming_state')==1 and p.get('system_id')==v['px4_system_id'] and p.get('pre_flight_checks_pass',False) and not p.get('failsafe',True) and d.get('land',{}).get('landed',False) and plat.get('connected',False) and not plat.get('armed',True) and not plat.get('offboard',True) and plat.get('status',{}).get('state')==PlatformStatus.DISARMED
            checks[ns+'/input_pairing']=i.get('image_pairs',0)>0 and i.get('odometry_count',0)>0 and i.get('offset') is not None and 0<=(sim-i.get('last_pair_ns',0))/1e9<=C['depth_sample_max_age_s'] and 0<=(sim-i.get('last_odom_ns',0))/1e9<=C['odometry_sample_max_age_s']
            checks[ns+'/depth_received']=d.get('depth',{}).get('bytes',0)>0
            checks[ns+'/native_waiting']=n.get('fsm')=='WAIT_TRIGGER' and n.get('frontier_points',0)>0 and not n.get('process_failure') and n.get('trigger_generation')==0
            checks[ns+'/adapter_ready_idle']=a.get('ready',False) and not a.get('active_history') and not a.get('goal_handle') and not a.get('pending') and a.get('context_id') is None
            checks[ns+'/as2_actions_discovered']=all(x.server_is_ready() for x in self.actions[ns])
            cli=self.param_clients[ns]
            if ns not in self.futures and cli.service_is_ready():self.futures[ns]=cli.call_async(GetParameters.Request(names=self.names))
            fut=self.futures.get(ns)
            if fut and fut.done() and ns not in self.parameters:
                self.parameters[ns]={k:message_to_ordereddict(x) for k,x in zip(self.names,fut.result().values)}
            pars=self.parameters.get(ns,{})
            expect={f'sdf_map.box_{edge}_{axis}':F['planning_box_enu'][j][k] for j,axis in enumerate('xyz') for k,edge in enumerate(('min','max'))}
            checks[ns+'/native_parameters']=all(pars.get(k,{}).get('double_value')==val for k,val in expect.items()) and pars.get('exploration.drone_num',{}).get('integer_value')==F['native_slots'] and pars.get('exploration.drone_id',{}).get('integer_value')==v['racer_id'] and pars.get('external_execution.enabled',{}).get('bool_value') is True
        graph=getattr(self,'peer_graph',{})
        if not graph or now-getattr(self,'graph_ts',0)>1:
            self.graph_ts=now
            for topic in ('/swarm_expl/drone_state','/multi_map_manager/chunk_stamps'):
                graph[topic]={kind:sorted({ep.node_name for ep in fn(topic)}) for kind,fn in
                             [('pub',self.get_publishers_info_by_topic),('sub',self.get_subscriptions_info_by_topic)]}
            self.peer_graph=graph
        expected={'racer_'+v['namespace'] for v in F['vehicles']}
        checks['native_peer_endpoints']=all(expected<=set(names) for endpoints in graph.values() for names in endpoints.values())
        return checks

rclpy.init();node=Probe();start=time.monotonic();ready_at=None;outcome={}
try:
    while True:
        rclpy.spin_once(node,timeout_sec=.03);checks=node.check()
        if node.armed:raise RuntimeError('armed observed; preserve services for recovery')
        if all(checks.values()):
            if ready_at is None:ready_at=time.monotonic()
            if time.monotonic()-ready_at>=10:break
        if time.monotonic()-start>F['readiness_timeout_s']:raise TimeoutError('readiness deadline: '+str([k for k,v in checks.items() if not v]))
    outcome['pass']=True
except Exception as exc:outcome.update({'pass':False,'error':str(exc)})
finally:
    node.sample()
    outcome['peer_graph']=getattr(node,'peer_graph',{})
    outcome.update(scope='real PC ground readiness only; no trigger/arm/flight',motion_requested=False,checks=node.check(),seconds=time.monotonic()-start,counts=dict(node.counts),rows=node.rows,parameters=node.parameters,peer_ids=sorted(node.peer_ids),map_stamp_senders=sorted(node.stamp_ids),vehicle_commands=node.commands,flight_config=C)
    outcome['pass']=outcome.get('pass',False) and all(outcome['checks'].values())
    (OUT/'result.json').write_text(json.dumps(outcome,indent=2));node.log.close();node.destroy_node();rclpy.shutdown()
print(json.dumps({'pass':outcome['pass'],'checks':outcome['checks'],'error':outcome.get('error')},indent=2))
raise SystemExit(0 if outcome['pass'] else 1)
