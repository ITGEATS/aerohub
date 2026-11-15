"""Independent raw-event audit of scoped fault recovery and healthy peer progress."""
import json
import math
from pathlib import Path
import sys
from audit_flight import audit as basic_audit
from audit_ego import authorized_holds,reference_errors,source_peak

def audit(root):
    common=basic_audit(root);checks=dict(common['checks']);metrics=dict(common['metrics'])
    read=lambda p:json.loads((root/p).read_text())
    report=read('result.json');case=report['case'];v=report['fault_vehicle'];i=int(v[-1]);peer=1-i;t=report['injection_wall']
    checks['criteria_frozen']=report['gates']==read('criteria.json')
    baseline=root/'baseline-audit.json'
    if not baseline.exists():baseline=root.parent/'mission-basic/stage-audit.json'
    checks['same_session_basic']=json.loads(baseline.read_text())['passed']
    all_rows={j:[json.loads(l) for l in (root/f'drone{j}/flight.jsonl').read_text().splitlines()] for j in (0,1)}
    rows=all_rows[i];states=[r for r in rows if 'ego_state' in r and r['wall']>=t]
    failed=[r for r in states if r['ego_state']['state']=='FAILED'];stopped=[r for r in failed if r['ego_state']['stop_confirmed']]
    metrics['detection_wall_s']=failed[0]['wall']-t if failed else math.inf
    metrics['stop_confirmation_wall_s']=stopped[0]['wall']-t if stopped else math.inf
    checks['detection_bound']=metrics['detection_wall_s']<=1.5;checks['stop_confirmation_bound']=metrics['stop_confirmation_wall_s']<=5
    checks['specific_failure']=bool(failed) and failed[0]['ego_state']['reason'].startswith(('CANCELED' if case=='cancel' else 'STALE_INPUT')+':')
    checks['closed_without_resume']=bool(failed) and all(r['ego_state']['state']=='FAILED' for r in states if r['wall']>=failed[0]['wall'])
    checks['no_new_segments']=bool(stopped) and all(r['ego_state']['revision']==stopped[0]['ego_state']['revision'] and r['ego_state']['counts']['segments']==stopped[0]['ego_state']['counts']['segments'] for r in stopped)
    final=report['scoped_states'][str(i)];checks['new_goal_rejected']=final.get('rejected',{}).get('goal',0)>=1
    odom=[r for r in rows if r.get('topic')=='odom'];pre=[r for r in odom if r['phase']=='ego' and r['wall']<t];after=[r for r in odom if r['wall']>=t and r['phase'] in ('ego','fault_hold')]
    summary=read(v+'/result.json')
    checks['moving_at_injection']=bool(pre) and pre[-1]['speed']>=.1 and math.dist(pre[-1]['position'],summary['ego_start_position'])>=.25
    peer_pre=[r for r in all_rows[peer] if r.get('topic')=='odom' and r['phase']=='ego' and r['wall']<t]
    peer_summary=read(f'drone{peer}/result.json')
    checks['peer_moving_at_injection']=bool(peer_pre) and peer_pre[-1]['speed']>=.1 and math.dist(peer_pre[-1]['position'],peer_summary['ego_start_position'])>=.25
    first=None;end=None
    for r in after:
        if r['speed']>.1:first=None;continue
        if first is None:first=r
        if r['sim']-first['sim']>=1:end=r;break
    metrics['physical_stop_wall_s']=first['wall']-t if end else math.inf
    metrics['residual_displacement_m']=max((math.dist(r['position'],report['pre_fault']['positions'][i]) for r in after if end and r['wall']<=end['wall']),default=math.inf)
    checks['physical_stop_bound']=metrics['physical_stop_wall_s']<=5;checks['residual_displacement']=metrics['residual_displacement_m']<=.75
    hold=[r for r in after if r['phase']=='fault_hold']
    metrics['post_stop_drift_m']=max((math.dist(r['position'],report['stopped_position']) for r in hold),default=math.inf)
    checks['hold_drift']=metrics['post_stop_drift_m']<=.3
    checks['hold_duration']=bool(hold) and hold[-1]['sim']-hold[0]['sim']>=3
    checks['no_old_references']=bool(hold) and not any(r.get('topic')=='reference_trajectory' and r['wall']>hold[0]['wall']+.6 and r['phase']=='fault_hold' for r in rows)
    action_rows=[r for r in rows if r.get('timed_action_status') and t<=r['wall']<min((x['wall'] for x in rows if x['phase']=='landing'),default=math.inf)]
    checks['execution_terminated']=bool(action_rows) and action_rows[-1]['timed_action_status'][-1] in (5,6) and report['terminal_action_status']==action_rows[-1]['timed_action_status']
    checks['behavior_idle']=any(r.get('timed_behavior_status')==0 and r['phase']=='fault_hold' for r in rows)
    checks['fault_hold_offboard']=any(r.get('topic')=='mode' and r['phase']=='fault_hold' for r in rows) and all(r['armed'] and r['offboard'] for r in rows if r.get('topic')=='mode' and r['phase']=='fault_hold')
    checks['land_after_stop']=bool(stopped) and min((r['wall'] for r in rows if r['phase']=='landing'),default=0)>stopped[0]['wall']
    for j in (0,1):
        name=f'drone{j}';log=all_rows[j];s=read(name+'/result.json');nav=[r for r in log if r.get('topic')=='odom' and r['wall']>=t and r['phase'] in ('ego','fault_hold')]
        origin_events=[r for r in log if 'takeoff_origin_at_action' in r]
        takeoff_odom=[r for r in log if r.get('topic')=='odom' and r['phase']=='taking_off']
        checks[name+'_action_time_takeoff_origin']=len(origin_events)==1 and bool(takeoff_odom) and origin_events[0]['takeoff_origin_at_action']==s['origin_enu'] and math.dist(takeoff_odom[0]['position'],s['origin_enu'])<=.05
        checks[name+'_live_navigation']=len(nav)>30 and max((b['wall']-a['wall'] for a,b in zip(nav,nav[1:])),default=math.inf)<1
        polys=read(name+'/polynomials.json');refs=read(name+'/references.json');segments=read(name+'/segments.json')
        errors=[max(e) for e in reference_errors(polys,refs,authorized_holds(polys,segments,s['goal_world_enu'],name))]
        metrics[name+'_pva_error']=max(errors,default=math.inf)
        checks[name+'_native_pva']=len(refs)>=20 and bool(polys and segments) and metrics[name+'_pva_error']<=1e-4
        checks[name+'_native_limits']=bool(polys) and all(source_peak(p,1)<=.2415+1e-8 and source_peak(p,2)<=.3675+1e-8 for p in polys)
        checks[name+'_flight_speed']=bool(nav) and max(r['speed'] for r in nav)<=.5
        checks[name+'_collision_checkpoints']='ego_collision_after' in s and s['ego_collision_before']==s['ego_collision_after']
        owner=[r['ego_state'] for r in log if 'ego_state' in r]
        checks[name+'_ownership']=bool(owner) and all(x['vehicles']==[name] and x['interface_scope']==name for x in owner)
    peer_rows=all_rows[peer];peer_states=[r for r in peer_rows if 'ego_state' in r and r['wall']>=t]
    checks['peer_never_failed']=bool(peer_states) and all(r['ego_state']['state']!='FAILED' for r in peer_states)
    ps=report['scoped_states'][str(peer)];checks['peer_original_goal_completed']=ps['state']=='SUCCEEDED' and ps['stop_confirmed'] and ps['counts']['goals']==1
    po=[r for r in peer_rows if r.get('topic')=='odom' and r['phase']=='ego'];goal=read(f'drone{peer}/result.json')['goal_enu']
    metrics['peer_goal_error_m']=math.dist(po[-1]['position'],goal) if po else math.inf
    checks['peer_goal_accuracy']=metrics['peer_goal_error_m']<=.25
    checks['peer_progress_after_fault']=bool(po) and math.dist(po[-1]['position'],report['pre_fault']['positions'][peer])>=.5
    graph=report['input_graph']
    checks['scoped_gate_graph']=all(graph[f'/acceptance/{v}/{s}']['publishers']==[dict(name='acceptance_fault_injector',namespace='/'+v)] and dict(name='ego_as2_adapter',namespace='/'+v) in graph[f'/acceptance/{v}/{s}']['subscribers'] for s in ('odom','points'))
    checks['original_navigation_unchanged']=all(graph[f'/{v}/navigation/{s}']['publishers']==[dict(name='navigation_inputs',namespace='/'+v)] for s in ('odom','points'))
    injector=report['final_injector'];events=injector['events'];checks['injector_owner']=injector['vehicle']==v
    if case=='cancel':
        checks['scoped_cancel_called']=any(r.get('service')==f'/{v}/ego/cancel' and r.get('success') for r in all_rows[0]);checks['no_unrequested_injection']=not events
    elif case=='planner_exit':
        config_path=root/'deployment-config.json'
        if not config_path.exists():config_path=root.parent/'config.json'
        config=json.loads(config_path.read_text());allowed=config['candidate']+'/install/' if i==0 else '/work/install/humble/'
        checks['owned_planner_exit']=len(events)==1 and events[0]['kind']=='planner_exit' and events[0]['signal']=='SIGKILL' and events[0]['vehicle']==v and events[0]['run_id']==config['run_id'] and events[0]['parent']>1 and bool(events[0]['ticks']) and events[0]['domain']==['ROS_DOMAIN_ID='+str(config['domain'])] and '__ns:=/'+v in events[0]['argv'] and events[0]['argv'][0].startswith(allowed)
    else:
        checks['only_requested_gate']=len(events)==2 and all(e['kind']==case and e['vehicle']==v for e in events)
        checks['blocked_restored']=len(events)==2 and events[0]['blocked'] and not events[1]['blocked']
        checks['no_blocked_forwarding']=len(events)==2 and events[0]['counts']['forwarded']==events[1]['counts']['forwarded']
        checks['live_input_dropped']=len(events)==2 and events[1]['counts']['dropped']-events[0]['counts']['dropped']>=10
        checks['live_input_restored']=len(events)==2 and injector['counts'][case]['forwarded']>=events[1]['counts']['forwarded']+10
    checks={k:bool(v) for k,v in checks.items()}
    result=dict(passed=all(checks.values()),passed_checks=sum(checks.values()),total_checks=len(checks),checks=checks,metrics=metrics,case=case,fault_vehicle=v,
        scope='Mixed two-UAV planner-boundary fault; healthy peer finishes; no AS2 localization-loss or network-loss claim')
    (root/'stage-audit.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
    result=audit(Path(sys.argv[1]));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
