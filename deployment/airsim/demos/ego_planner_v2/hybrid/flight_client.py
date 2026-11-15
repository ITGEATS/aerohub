"""Workstation-only mixed-fleet client; reuse real native action/flight checks."""
import json
import math
from pathlib import Path
import sys
import time
sys.path.insert(0,'/input')
from dual_flight_mission import Fleet, GoalSet, Trigger, String, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from traj_utils.msg import MINCOTraj
from uav_planner_interfaces.msg import TimedTrajectorySegment
from ego_reference_check import compare_references, segment_rows


class PerVehicleGoals:
    def __init__(self,publishers):self.publishers=publishers
    def publish(self,msg):self.publishers[msg.drone_id].publish(msg)
    def get_subscription_count(self):return min(p.get_subscription_count() for p in self.publishers)


class MixedFleet(Fleet):
    def __init__(self,stage):
        super().__init__('dual' if stage=='short' else 'basic')
        self.requested_stage=stage
        self.minimum_initial_separation=3.5  # nominal 4 m, with the existing 0.5 m alignment gate
        self.scoped_states={};self.segments={0:[],1:[]};self.broadcasts=[]
        n=self.nodes[0]
        n.destroy_publisher(self.goal_pub);n.destroy_client(self.cancel_client);n.destroy_subscription(self.status_sub)
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub=PerVehicleGoals([n.create_publisher(GoalSet,f'/drone{i}/ego/goal',qos) for i in (0,1)])
        self.cancel_clients=[n.create_client(Trigger,f'/drone{i}/ego/cancel') for i in (0,1)]
        for i in (0,1):
            n.keep.append(n.create_subscription(String,f'/drone{i}/ego/status',lambda m,i=i:self.scoped_status(i,m),10))
            n.keep.append(n.create_subscription(TimedTrajectorySegment,f'/drone{i}/planner/trajectory',lambda m,i=i:self.segments[i].append(m),10))
            self.nodes[i].allow_peer_armed=True
        n.keep.append(n.create_subscription(MINCOTraj,'/ego_swarm/trajectory',self.broadcast,100))
        self.report.update(stage=stage,topology='RoboPi Drone0 compute + PC Drone1 compute; PC ROS mission only')
        self.report['gates']['minimum_initial_separation_m']=3.5
        (self.out/'criteria.json').write_text(json.dumps(self.report['gates'],indent=2))

    def scoped_status(self,i,msg):
        state=json.loads(msg.data);self.scoped_states[i]=state
        self.nodes[i].event(ego_state=state)
        if state.get('vehicles')!=[f'drone{i}'] or state.get('interface_scope')!=f'drone{i}':
            self.nodes[i].error='Adapter ownership mismatch'
        rows=list(self.scoped_states.values())
        name='FAILED' if any(s['state']=='FAILED' for s in rows) else (
            rows[0]['state'] if len(rows)==2 and rows[0]['state']==rows[1]['state'] else 'MIXED')
        self.ego_state=dict(state=name,reason='; '.join(s.get('reason','') for s in rows),
            stop_confirmed=len(rows)==2 and all(s.get('stop_confirmed') for s in rows),
            completed=[v for s in rows for v in s.get('completed',[])])
        self.ego_state['counts']={key:sum(s.get('counts',{}).get(key,0) for s in rows)
            for key in {key for s in rows for key in s.get('counts',{})}}

    def broadcast(self,msg):
        row=dict(wall=time.time(),message=message_to_ordereddict(msg))
        self.broadcasts.append(row);self.nodes[0].event(native_minco=row)

    def wait(self,predicate,timeout=15):
        def checked():
            if not self.recovering and (self.out/'recovery.request').exists():
                raise RuntimeError('Deployment requested recovery: '+(self.out/'recovery.request').read_text())
            return predicate()
        return super().wait(checked,timeout)

    def prepare(self):
        super().prepare()
        self.wait(lambda:len(self.scoped_states)==2 and all(s['state']=='WAITING' and s['counts']['goals']==0 for s in self.scoped_states.values()),20)
        expected=[('ego_planner','/drone0'),('ego_planner','/drone1')]
        self.report['native_minco_discovery']=[]
        def native_graph_ready():
            graph=self.nodes[0].get_publishers_info_by_topic('/ego_swarm/trajectory')
            snapshot=[dict(node=p.node_name,namespace=p.node_namespace,gid=list(p.endpoint_gid)) for p in graph]
            history=self.report['native_minco_discovery']
            if not history or history[-1]['publishers']!=snapshot:
                history.append(dict(wall=time.time(),publishers=snapshot))
                self.nodes[0].event(native_minco_discovery=snapshot)
            if sorted((p.node_name,p.node_namespace) for p in graph)!=expected:return False
            self.report['native_minco_publishers']=snapshot
            return True
        # A new mission participant can receive transient-local status before
        # endpoint discovery completes. Require the exact graph within 5 seconds.
        self.wait(native_graph_ready,5)
        if self.requested_stage=='short':
            prerequisite=json.loads(Path('/baseline/stage-audit.json').read_text())
            if not prerequisite['passed']:raise RuntimeError('Independent mixed takeoff/landing audit required')

    def audit(self):
        if self.requested_stage!='short':return
        for n in self.nodes:
            refs=[dict(stamp=r['stamp_ns'],p=r['position'],v=r['velocity'],a=r['acceleration']) for r in n.references]
            error,holds=compare_references(n.polys,refs,self.segments[n.drone_id],f'drone{n.drone_id}',[0,0,0],n.world(n.target))
            n.report.update(max_polynomial_reference_error=error,explicit_hold_samples=holds)
            if error>1e-4:raise RuntimeError(f'{n.vehicle_name} native/reference PVA mismatch: {error}')
        if {r['message']['drone_id'] for r in self.broadcasts}!={0,1}:raise RuntimeError('Missing bidirectional native MINCO broadcasts')

    def recover(self):
        self.recovering=True
        for n in self.nodes:n.recovering=True
        for i,client in enumerate(self.cancel_clients):
            if self.nodes[i].phase!='ego':continue
            try:
                self.wait(client.service_is_ready,5)
                future=client.call_async(Trigger.Request());self.wait(future.done,5)
                self.wait(lambda:self.scoped_states.get(i,{}).get('stop_confirmed'),10)
                self.report['recovery'].append(dict(drone=i,ego_stop=True))
            except Exception as exc:self.report['recovery'].append(dict(drone=i,ego_stop_error=repr(exc)))
        for n in self.nodes:
            try:n.recover()
            except Exception as exc:n.report['recovery_error']=repr(exc)
            self.report['recovery'].append(dict(drone=n.drone_id,steps=n.report['recovery'],error=n.report.get('recovery_error')))

    def finish(self):
        self.report['scoped_states']=self.scoped_states
        (self.out/'native-minco.json').write_text(json.dumps(self.broadcasts,indent=2))
        for n in self.nodes:
            (n.out/'segments.json').write_text(json.dumps(segment_rows(self.segments[n.drone_id]),indent=2))
        super().finish()


if __name__=='__main__':
    rclpy.init();fleet=MixedFleet(sys.argv[1])
    try:fleet.run()
    except BaseException as exc:
        fleet.report['passed']=False;fleet.report['error']=repr(exc)
        try:fleet.recover()
        except BaseException as recovery:fleet.report['recovery_error']=repr(recovery)
    finally:fleet.finish();rclpy.try_shutdown()
    raise SystemExit(0 if fleet.report['passed'] else 1)
