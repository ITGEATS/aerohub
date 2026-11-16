"""Independent raw-log audit; no mission or Adapter conversion imports."""
import json
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,'/single')
from audit_ego import sample,source_peak

def audit(root):
    read=lambda p:json.loads((root/p).read_text())
    report=read('result.json');checks={};metrics={}
    checks['client_success']=report['passed'] and read('client-exit.json')['returncode']==0
    checks['no_recovery']=not report['recovery'] and 'error' not in report and 'recovery_error' not in report
    checks['both_disarmed']=report['both_disarmed']
    clocks=[json.loads(l) for l in (root/'clock-offsets.jsonl').read_text().splitlines()]
    metrics['max_clock_bound_s']=max((abs(r['offset'])+r['rtt']/2 for r in clocks),default=math.inf)
    checks['runtime_clock']=metrics['max_clock_bound_s']<=.5 and not (root/'recovery.request').exists()
    tracks={};scenes={}
    for i in (0,1):
        name=f'drone{i}';summary=read(name+'/result.json')
        rows=[json.loads(l) for l in (root/name/'flight.jsonl').read_text().splitlines()]
        checks[name+'_native_actions']=([a['result'] for a in summary['actions']]==['takeoff_success','land_success'] and all(a['success'] and a['status']==4 for a in summary['actions']))
        checks[name+'_arm_offboard_acks']=[(c['service'],c['value'],c['ack']) for c in summary['commands']]==[('set_arming_state',True,0),('set_offboard_mode',True,0)]
        modes=[r for r in rows if r.get('topic')=='mode'];platforms=[r for r in rows if r.get('topic')=='platform']
        checks[name+'_disarmed_boundaries']=bool(modes and platforms) and not modes[0]['armed'] and not modes[-1]['armed'] and not platforms[-1]['armed']
        hold=[r for r in rows if r.get('topic')=='odom' and r['phase']=='holding'];origin=summary['origin_enu']
        checks[name+'_hover']=len(hold)>=20 and hold[-1]['sim']-hold[0]['sim']>=4.9 and all(abs(r['position'][2]-origin[2]-2)<=.3 and math.dist(r['position'][:2],origin[:2])<=.5 and r['speed']<=.3 for r in hold)
        checks[name+'_airborne_control']=any(r['armed'] for r in modes) and all(r['armed'] and r['offboard'] for r in modes if r['phase'] in ('taking_off','settling','holding','ego'))
        odom=[r for r in rows if r.get('topic')=='odom']
        checks[name+'_envelope']=all(r['speed']<=2 and all(math.isfinite(x) for x in r['position']) for r in odom)
        scene=[r for r in rows if 'scene_pose' in r and 'truth' in r]
        checks[name+'_scene_landing']=bool(scene) and abs(scene[-1]['scene_pose']['position']['z_val']-summary['truth_origin_ned'][2])<=.2
        checks[name+'_physics_landing']=max((abs(r['scene_pose']['position']['z_val']-r['truth']['position']['z_val']) for r in scene if r['phase']=='landing'),default=math.inf)<=.3
        checks[name+'_downstream']=sum(r.get('topic')=='px4_setpoint' for r in rows)>=30 and sum(r.get('topic') in ('actuator','actuator_twist') for r in rows)>=30
        tracks[i]=[(r['sim'],[r['position'][0]-4*i,*r['position'][1:]]) for r in odom]
        scenes[i]=[(r['sim'],[r['scene_pose']['position']['y_val']-4*i,r['scene_pose']['position']['x_val'],-r['scene_pose']['position']['z_val']]) for r in scene]
        if report['stage'] in ('short','crossing'):
            polys=read(name+'/polynomials.json');refs=read(name+'/references.json');segments=read(name+'/segments.json');goal=summary['goal_world_enu']
            errors=[];holds=[]
            for p in polys:
                end=p['start_ns']+round(sum(p['duration'])*1e9);pva=[sample(p,end,d) for d in range(3)]
                if math.dist(pva[0],goal)>=.03 or max(math.dist(x,[0,0,0]) for x in pva[1:])>=1e-4:continue
                for seg in segments:
                    if not seg['is_final']:continue
                    for v in seg['vehicles']:
                        if v['vehicle_id']!=name:continue
                        points=v['points'];stop=seg['start_ns']+points[-1]['t_ns']
                        if not end<stop<=end+100_000_000:continue
                        # The native endpoint knot preserves numerical P/V/A;
                        # the additional stationary point explicitly emits zero V/A.
                        if math.dist(points[-1]['p'],pva[0])>1e-9 or any(math.dist(points[-1][k],[0,0,0])>1e-12 for k in ('v','a')):continue
                        if any(seg['start_ns']+q['t_ns']==end and max(math.dist(q[k],x) for k,x in zip(('p','v','a'),pva))<=1e-9 for q in points):holds.append((end,stop,[pva[0],[0,0,0],[0,0,0]]))
            for r in refs:
                options=[];stamp=r['stamp_ns']
                for p in polys:
                    if p['start_ns']<=stamp<=p['start_ns']+round(sum(p['duration'])*1e9):
                        options.append(max(math.dist(sample(p,stamp,d),r[k]) for d,k in enumerate(('position','velocity','acceleration'))))
                options.extend(max(math.dist(x,r[k]) for x,k in zip(pva,('position','velocity','acceleration'))) for start,end,pva in holds if start<stamp<=end)
                errors.append(min(options,default=math.inf))
            metrics[name+'_pva_error']=max(errors,default=math.inf)
            checks[name+'_native_pva']=len(refs)>=20 and bool(polys and segments) and metrics[name+'_pva_error']<=1e-4
            checks[name+'_native_limits']=bool(polys) and all(source_peak(p,1)<=.2415+1e-8 and source_peak(p,2)<=.3675+1e-8 for p in polys)
            flight=[r for r in odom if r['phase']=='ego']
            checks[name+'_goal_and_speed']=bool(flight) and math.dist(flight[-1]['position'],summary['goal_enu'])<=.25 and max(r['speed'] for r in flight)<=.5
            checks[name+'_no_execution_collision']=summary['ego_collision_before']==summary['ego_collision_after']
            state=report['scoped_states'][str(i)]
            checks[name+'_scoped_success']=state['state']=='SUCCEEDED' and state['stop_confirmed'] and state['vehicles']==[name] and state['counts']['goals']==1
            messages=[r['message'] for r in read('native-minco.json') if r['message']['drone_id']==i]
            checks[name+'_native_broadcast_matches_poly']=any(any(m['traj_id']==p['traj_id'] and abs(m['start_time']['sec']*1e9+m['start_time']['nanosec']-p['start_ns'])<=1024 and len(m['duration'])==len(p['duration']) and max(abs(a-b) for a,b in zip(m['duration'],p['duration']))<1e-8 for p in polys) for m in messages)
    for label,data in [('odom',tracks),('scene',scenes)]:
        a,b=data[0],data[1];distances=[]
        for stamp,pos in a:
            other=min(b,key=lambda r:abs(r[0]-stamp))
            if abs(other[0]-stamp)<=.5:distances.append(math.dist(pos,other[1]))
        metrics['minimum_'+label+'_separation_m']=min(distances,default=-1)
        checks[label+'_separation']=len(distances)>=20 and min(distances)>=2.
    if report['stage'] in ('short','crossing'):checks['simultaneous_motion']=report['simultaneous_motion_sim_s']>=.5
    result=dict(passed=all(checks.values()),passed_checks=sum(checks.values()),total_checks=len(checks),checks=checks,metrics=metrics,
        scope='Independent mixed-flight raw data; native peer acceptance and lifecycle require session audit')
    (root/'stage-audit.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':
    result=audit(Path(sys.argv[1]));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
