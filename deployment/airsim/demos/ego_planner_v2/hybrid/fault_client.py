"""One owner fails closed while its peer completes the original native goal."""
import json
import math
from pathlib import Path
import sys
import time
sys.path.insert(0,'/input')
from fault_flight_mission import FaultMission as PreviousFault,OffboardControlMode,qos_profile_sensor_data,GoalStatusArray,BehaviorStatus,ControlMode,SetControlMode
from flight_client import MixedFleet,GoalSet,String,rclpy

class MixedFault(PreviousFault,MixedFleet):
    def __init__(self):
        self.config=json.loads(Path('/session/config.json').read_text())
        self.target_id=int(self.config['fault_vehicle'][-1]);self.peer_id=1-self.target_id
        self.case=self.config['fault_case'];self.expect_failure=False;self.injected=None
        self.injector={};self.timeline=[];self.next_sample=0.;self.action_status={};self.behavior={};self.px4_modes={}
        MixedFleet.__init__(self,'short')
        self.check_independent_route_tube=False
        self.report.update(stage='fault',case=self.case,fault_vehicle=self.config['fault_vehicle'],
            recovery_policy='fault owner stop/hold; healthy peer completes original goal; both native Land',
            input_fault_scope='selected owner planner input only; AS2 localization stays live')
        self.report['gates'].update(detection_wall_s=1.5,stop_confirmation_wall_s=5.,physical_stop_wall_s=5.,
            stop_speed_m_s=.1,stable_sim_s=1.,post_stop_hold_sim_s=3.,max_residual_displacement_m=.75,
            max_post_stop_drift_m=.3,goal_distance_m=2.,max_horizontal_excursion_m=4.,
            expected_failure='CANCELED' if self.case=='cancel' else 'STALE_INPUT',no_old_task_resume=True,
            planner_input_timeout_wall_s=.75,peer_goal_timeout_wall_s=60.,peer_goal_error_m=.25)
        (self.out/'criteria.json').write_text(json.dumps(self.report['gates'],indent=2))
        node=self.nodes[0];self.keep=[node.create_subscription(String,'/acceptance/'+self.config['fault_vehicle']+'/injector_status',self.injector_status,10)]
        for i,n in enumerate(self.nodes):
            n.report['gates']['max_horizontal_excursion_m']=4.
            def action(m,i=i):
                self.action_status[i]=[s.status for s in m.status_list];self.nodes[i].event(timed_action_status=self.action_status[i])
            def behavior(m,i=i):
                self.behavior[i]=m.status;self.nodes[i].event(timed_behavior_status=m.status)
            self.keep.extend([node.create_subscription(OffboardControlMode,f'/drone{i}/fmu/in/offboard_control_mode',lambda m,i=i:self.px4_modes.update({i:m}),qos_profile_sensor_data),
                node.create_subscription(GoalStatusArray,f'/drone{i}/follow_timed_path/_action/status',action,10),
                node.create_subscription(BehaviorStatus,f'/drone{i}/follow_timed_path/_behavior/behavior_status',behavior,10)])

    def scoped_status(self,i,msg):
        MixedFleet.scoped_status(self,i,msg)
        s=self.scoped_states[i]
        if self.injected is not None and i==self.target_id:
            elapsed=time.monotonic()-self.injected
            if s['state']=='FAILED':self.report.setdefault('detection_wall_s',elapsed)
            if s['stop_confirmed']:self.report.setdefault('stop_confirmation_wall_s',elapsed)
        if s['state']=='FAILED' and (i!=self.target_id or not self.expect_failure):
            self.nodes[i].error='Unexpected owner failure: '+s.get('reason','')

    def wait(self,predicate,timeout=15):
        def checked():
            if not self.recovering:
                if (self.out/'recovery.request').exists():raise RuntimeError((self.out/'recovery.request').read_text())
                for n in self.nodes:
                    if n.phase=='fault_hold' and not (n.latest['mode'].flag_armed and n.latest['mode'].flag_control_offboard_enabled):raise RuntimeError('Fault hold lost offboard')
            return predicate()
        return PreviousFault.wait(self,checked,timeout)

    def prepare(self):
        MixedFleet.prepare(self)
        self.wait(lambda:all(self.injector.get('counts',{}).get(k,{}).get('forwarded',0)>=10 for k in ('cloud','odom')),15)
        v=self.config['fault_vehicle']
        topics=[f'/{v}/navigation/{s}' for s in ('odom','points')]+[f'/acceptance/{v}/{s}' for s in ('odom','points')]
        self.report['input_graph']={t:dict(publishers=[dict(name=e.node_name,namespace=e.node_namespace) for e in self.nodes[0].get_publishers_info_by_topic(t)],
            subscribers=[dict(name=e.node_name,namespace=e.node_namespace) for e in self.nodes[0].get_subscriptions_info_by_topic(t)]) for t in topics}
        for t in topics[2:]:
            graph=self.report['input_graph'][t]
            assert graph['publishers']==[dict(name='acceptance_fault_injector',namespace='/'+v)]
            assert dict(name='ego_as2_adapter',namespace='/'+v) in graph['subscribers']
        for i,n in enumerate(self.nodes):
            c=n.create_client(SetControlMode,f'/drone{i}/controller/set_control_mode');self.wait(c.service_is_ready,5)
            f=c.call_async(SetControlMode.Request(control_mode=ControlMode(control_mode=ControlMode.HOVER)))
            self.wait(f.done,5);assert f.result().success
            self.wait(lambda:n.latest['controller'].output_control_mode.control_mode==ControlMode.SPEED,5)

    def ego(self):
        starts,goals,clear=self.routes()
        self.report['routes']=dict(starts_world_enu=starts.tolist(),goals_world_enu=goals.tolist(),clearances_m=clear)
        stationary=[None]
        def stable_before_goals():
            if any(n.speed()>.1 for n in self.nodes):stationary[0]=None;return False
            if stationary[0] is None:stationary[0]=self.nodes[0].now_s()
            return self.nodes[0].now_s()-stationary[0]>=1.
        self.wait(stable_before_goals,10)
        # Check both stationary boundaries before publishing either motion goal.
        for n in self.nodes:n.report['ego_collision_before']=n.collision_checkpoint('before_ego')
        for i,n in enumerate(self.nodes):
            n.target=goals[i].copy();n.target[0]+=4*i;n.target=n.target.tolist()
            n.report.update(ego_start_position=n.position(),goal_enu=n.target,goal_world_enu=goals[i].tolist())
            n.start_counts=n.counts.copy();n.phase='ego'
            self.goal_pub.publish(GoalSet(drone_id=i,goal=goals[i].tolist()))
        self.wait(lambda:all(self.scoped_states[i]['state']=='RUNNING' and len(n.references)>=20 and n.speed()>=.1 and math.dist(n.position(),n.report['ego_start_position'])>=.25 for i,n in enumerate(self.nodes)),30)
        self.report['pre_fault']=dict(states=self.scoped_states.copy(),positions=[n.position() for n in self.nodes],injector=self.injector.copy())
        self.expect_failure=True;self.injected=time.monotonic();self.report.update(injection_wall=time.time(),injection_sim=self.nodes[0].now_s())
        i=self.target_id;n=self.nodes[i];peer=self.nodes[self.peer_id]
        prefix='/acceptance/'+self.config['fault_vehicle']
        service={'cancel':f'/drone{i}/ego/cancel','planner_exit':prefix+'/exit_planner','cloud':prefix+'/block_cloud','odom':prefix+'/block_odom'}[self.case]
        self.report['injection_response']=self.invoke(service,True if self.case in ('cloud','odom') else None)
        state=lambda:self.scoped_states[i]
        self.wait(lambda:state()['state']=='FAILED',1.5)
        assert state()['reason'].startswith(self.report['gates']['expected_failure']+':') and self.report['detection_wall_s']<=1.5
        self.wait(lambda:state()['stop_confirmed'],5);assert self.report['stop_confirmation_wall_s']<=5
        stable=[None]
        def stopped():
            if n.speed()>.1:stable[0]=None;return False
            if stable[0] is None:stable[0]=(n.now_s(),time.monotonic()-self.injected)
            return n.now_s()-stable[0][0]>=1.
        self.wait(stopped,5)
        self.report['physical_stop_wall_s']=stable[0][1];assert stable[0][1]<=5
        self.report['stopped_position']=n.position()
        self.report['residual_displacement_m']=max(math.dist(s['positions'][i],self.report['pre_fault']['positions'][i]) for s in self.timeline)
        assert self.report['residual_displacement_m']<=.75
        n.phase='fault_hold';start=n.now_s();self.wait(lambda:n.now_s()-start>=.5,5)
        count=n.counts.get('reference_trajectory',0);revision=state()['revision'];segments=state()['counts']['segments'];hold_start=len(self.timeline)
        start=n.now_s();self.wait(lambda:n.now_s()-start>=3,10)
        assert self.behavior[i]==BehaviorStatus.IDLE and self.action_status[i][-1] in (5,6)
        self.report['terminal_action_status']=self.action_status[i]
        if self.case in ('cloud','odom'):
            self.report['blocked_input_evidence']=self.injector.copy();counts=self.injector['counts'][self.case]
            assert counts['dropped']>=10
            before=counts['forwarded'];self.invoke(service,False)
            self.wait(lambda:self.injector['counts'][self.case]['forwarded']>=before+10,5)
        rejected=state().get('rejected',{}).get('goal',0)
        self.goal_pub.publish(GoalSet(drone_id=i,goal=goals[i].tolist()))
        self.wait(lambda:state().get('rejected',{}).get('goal',0)>rejected,5)
        start=n.now_s();self.wait(lambda:n.now_s()-start>=2,8)
        # Peer is never canceled: it must finish the originally issued goal.
        self.wait(lambda:self.scoped_states[self.peer_id]['state']=='SUCCEEDED' and self.scoped_states[self.peer_id]['stop_confirmed'],60)
        stationary=[None]
        def both_stopped():
            if any(owner.speed()>.1 for owner in self.nodes):stationary[0]=None;return False
            if stationary[0] is None:stationary[0]=n.now_s()
            return n.now_s()-stationary[0]>=1.
        self.wait(both_stopped,5)
        assert self.scoped_states[self.peer_id]['counts']['goals']==1 and math.dist(peer.position(),peer.target)<=.25
        assert state()['state']=='FAILED' and state()['stop_confirmed'] and state()['revision']==revision and state()['counts']['segments']==segments
        assert n.counts.get('reference_trajectory',0)==count
        self.report['post_stop_drift_m']=max(math.dist(s['positions'][i],self.report['stopped_position']) for s in self.timeline[hold_start:]);assert self.report['post_stop_drift_m']<=.3
        self.report.update(no_old_task_resume=True,fault_assertions_passed=True,final_injector=self.injector.copy(),peer_goal_error_m=math.dist(peer.position(),peer.target))
        for owner in self.nodes:
            owner.report['ego_collision_after']=owner.collision_checkpoint('after_fault_and_peer_stop')
            assert owner.report['ego_collision_after']==owner.report['ego_collision_before']

    def actions(self,kind):
        if kind=='takeoff':
            self.wait(lambda:all(time.monotonic()-n.received.get('odom',0)<.1 for n in self.nodes),5)
            for n in self.nodes:
                n.report['prearm_origin_enu']=n.origin
                n.origin=n.position();n.report['origin_enu']=n.origin
                n.event(takeoff_origin_at_action=n.origin,prearm_origin_enu=n.report['prearm_origin_enu'])
        return super().actions(kind)

    def audit(self):MixedFleet.audit(self)
    def land(self):
        super().land()
        start=self.nodes[0].now_s();self.wait(lambda:self.nodes[0].now_s()-start>=1.,5)
    def recover(self):MixedFleet.recover(self)
    def finish(self):
        (self.out/'fault-timeline.json').write_text(json.dumps(self.timeline,indent=2,default=float))
        self.report['final_injector']=self.injector;MixedFleet.finish(self)

if __name__=='__main__':
    rclpy.init();mission=MixedFault()
    try:mission.run()
    except BaseException as exc:
        import traceback;traceback.print_exc();mission.report.update(passed=False,error=repr(exc))
        try:mission.recover()
        except BaseException as recovery:mission.report['recovery_error']=repr(recovery)
    finally:mission.finish();rclpy.try_shutdown()
    raise SystemExit(0 if mission.report['passed'] else 1)
