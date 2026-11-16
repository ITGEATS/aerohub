"""Crossing-only checks added to the independent scoped flight/PVA audit."""
import json
import math
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit_flight import audit as common_audit

def audit(root):
    common=common_audit(root)
    report=json.loads((root/'result.json').read_text());assert report['stage']=='crossing'
    checks=dict(common['checks']);metrics=dict(common['metrics']);tracks={};scenes={};begins=[];ends=[]
    starts=np.array(report['routes']['starts_world_enu']);goals=np.array([g['world'] for g in report['routes']['goals']])
    nominal=float(np.linalg.norm((starts[0]+goals[0]-starts[1]-goals[1])/2))
    checks['nominal_collision_course']=nominal<=.2 and abs(starts[0,2]-starts[1,2])<=.2
    checks['simultaneous_goal_dispatch']=max(report['goal_publish_times'])-min(report['goal_publish_times'])<=.1
    detours=[]
    for i in (0,1):
        name=f'drone{i}';summary=json.loads((root/name/'result.json').read_text())
        rows=[json.loads(l) for l in (root/name/'flight.jsonl').read_text().splitlines()]
        odom=[r for r in rows if r.get('topic')=='odom' and r['sim']>0 and abs(r['sim']-r['stamp'])<1]
        scene=[r for r in rows if 'scene_pose' in r and r['sim']>0]
        tracks[i]=np.array([[r['sim'],r['position'][0]-4*i,*r['position'][1:],r['speed']] for r in odom])
        scenes[i]=np.array([[r['sim'],r['scene_pose']['position']['y_val']-4*i,r['scene_pose']['position']['x_val'],-r['scene_pose']['position']['z_val']] for r in scene])
        encounter=[r for r in odom if r['phase']=='ego'];begins.append(min(r['sim'] for r in encounter));ends.append(max(r['sim'] for r in encounter))
        actual=np.array([r['position'] for r in encounter]);start=np.array(summary['ego_start_position']);goal=np.array(summary['goal_enu']);direction=goal-start
        along=np.clip((actual-start)@direction/np.dot(direction,direction),0,1)
        detours.append(float(np.linalg.norm(actual-(start+along[:,None]*direction),axis=1).max()))
        points=np.load(root/name/'preflight-cloud.npy');projection=np.clip((points-start)@direction/np.dot(direction,direction),0,1)
        clearance=float(np.linalg.norm(points-(start+projection[:,None]*direction),axis=1).min())
        height=[summary['truth_origin_ned'][2]-r['scene_pose']['position']['z_val'] for r in scene if r['phase']=='ego']
        checks[name+'_static_route_clearance']=clearance>=.6
        checks[name+'_height_and_excursion']=max(height)<3.2 and min(height)>-.4 and np.linalg.norm(actual[:,:2]-np.array(summary['origin_enu'])[:2],axis=1).max()<=9
        checks[name+'_offboard_mode_restored']=summary.get('post_offboard_mode_restored',{}).get('px4_setpoints',0)>=20
        checks[name+'_measured_peer_mask']=report['scoped_states'][str(i)]['counts'].get('peer_points_removed_'+name,0)>0
        checks[name+'_native_parameters']=report['native_parameters'][str(i)]==[1.,i,True,1.2,2.6]
        metrics[name+'_detour_m']=detours[-1];metrics[name+'_straight_cloud_clearance_m']=clearance
    begin=max(begins);end=min(ends)
    distances={};grids={};moving=0
    for label,data,step,limit in [('odom',tracks,.02,.1),('scene',scenes,.1,.6)]:
        missing=[]
        for i,t in data.items():
            missing.extend(dict(drone=i,start=float(a),end=float(b),gap=float(b-a)) for a,b in zip(t[:-1,0],t[1:,0]) if b-a>limit and b>begin and a<end)
        checks[label+'_encounter_complete_coverage']=not missing
        metrics[label+'_encounter_gaps']=missing
        grid=np.arange(begin,end,step);pos=[np.column_stack([np.interp(grid,t[:,0],t[:,k]) for k in range(1,t.shape[1])]) for t in data.values()]
        dist=np.linalg.norm(pos[0][:,:3]-pos[1][:,:3],axis=1)
        checks[label+'_aligned_encounter_separation']=len(grid)>20 and dist.min()>=2.
        metrics[label+'_aligned_minimum_separation_m']=float(dist.min());grids[label]=grid;distances[label]=dist
        if label=='odom':moving=float(((pos[0][:,3]>.05)&(pos[1][:,3]>.05)).sum()*step)
    checks['actual_avoidance_detour']=max(detours)>=.5
    checks['concurrent_motion_2s']=moving>=2
    metrics.update(nominal_synchronous_separation_m=nominal,aligned_simultaneous_motion_s=moving)
    # NumPy comparisons return bool_, whose sum becomes int64; normalize the
    # verdicts before exporting JSON, preserving their truth values.
    checks={key:bool(value) for key,value in checks.items()}
    result=dict(passed=all(checks.values()),passed_checks=sum(checks.values()),total_checks=len(checks),checks=checks,metrics=metrics,
        scope='Mixed same-height-goal crossing, native PVA execution and fully covered sampled encounter; peer optimization consumption audited in session logs')
    (root/'stage-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    for i,t in tracks.items():
        color=('tab:blue','tab:orange')[i]
        axes[0].plot(t[:,1],t[:,2],color=color,label=('RoboPi Drone0','PC Drone1')[i])
        axes[0].plot([starts[i,0],goals[i,0]],[starts[i,1],goals[i,1]],color=color,ls=':')
    for label in distances:axes[1].plot(grids[label]-begin,distances[label],label=label)
    axes[1].axhline(2,color='red',ls=':',label='2 m gate')
    axes[0].set(xlabel='World ENU east (m)',ylabel='World ENU north (m)',title='Actual tracks and direct conflict routes');axes[0].axis('equal')
    axes[1].set(xlabel='EGO encounter time (s)',ylabel='Separation (m)',title='Time-aligned sampled separation')
    for ax in axes:ax.grid(alpha=.3);ax.legend()
    fig.savefig(root/'crossing-tracks.png',dpi=160)
    return result

if __name__=='__main__':
    result=audit(Path(sys.argv[1]));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
