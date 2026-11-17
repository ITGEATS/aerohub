"""Offline native peer-consumption, camera, deployment and cleanup evidence."""
import hashlib
import json
from pathlib import Path
import re
import sys
import tarfile
import math

def audit(out):
    read=lambda name:json.loads((out/name).read_text())
    checks={};metrics={}
    stage=read('config.json').get('mission_stage','short')
    for task in ('basic',stage):checks[task+'_independent_audit']=read('mission-'+task+'/stage-audit.json')['passed']
    obs=read('observer.json')
    checks['ground_observation']=obs['duration']>=40 and all(r['count']>=30 and r['max_gap']<.75 and r['receipt_age']<.75 and len(obs['graph'][t]['publishers'])==1 for t,r in obs['topics'].items())
    archive=max(out.glob('board-*.tar.gz'))
    checks['board_archive_hash']=hashlib.sha256(archive.read_bytes()).hexdigest()==json.loads(archive.with_suffix('.sha256.json').read_text())['sha256']
    with tarfile.open(archive) as tar:
        def load(name):return tar.extractfile(next(m for m in tar.getmembers() if m.name.removeprefix('./')==name)).read().decode()
        logs={0:load('ego.log')};as2={0:load('as2.log')};stops={0:json.loads(load('stopped.json'))}
    logs[1]=(out/'pc-runtime/ego.log').read_text();as2[1]=(out/'pc-runtime/as2.log').read_text();stops[1]=read('pc-runtime/stopped.json')
    messages=[r['message'] for r in read('mission-'+stage+'/native-minco.json')]
    for i in (0,1):
        accepts=[dict(self=int(a),peer=int(b),traj=int(c),start=float(d)) for a,b,c,d in re.findall(r'SWARM_ACCEPT self=(\d+) peer=(\d+) traj=(\d+) start=([\d.]+)',logs[i])]
        matches=[a for a in accepts if a['self']==i and a['peer']==1-i and any(m['drone_id']==a['peer'] and m['traj_id']==a['traj'] and abs(m['start_time']['sec']+m['start_time']['nanosec']/1e9-a['start'])<=1e-6 for m in messages)]
        metrics[f'drone{i}_native_peer_accepts']=accepts
        checks[f'drone{i}_actual_native_peer_consumption']=len(matches)>0
        if stage=='crossing':
            optimized=re.findall(rf'SWARM_OPT self={i} peer={1-i} traj=(\d+) min_ellip=([\d.]+) success=1',logs[i])
            consumed=sorted({a['traj'] for a in matches}&{int(t) for t,_ in optimized})
            metrics[f'drone{i}_successful_neighbor_optimizations']=len(optimized)
            metrics[f'drone{i}_optimized_received_trajectory_ids']=consumed
            checks[f'drone{i}_peer_used_in_successful_optimization']=len(consumed)>0
        safe=stops[i]['safe_state']
        fault_owner=stage=='fault' and read('config.json')['fault_vehicle']==f'drone{i}'
        checks[f'drone{i}_safe_stop']=safe['fresh'] and safe['armed'] is False and safe['px4_armed'] is False and safe['behavior_idle'] and safe['ego_state']==('FAILED' if fault_owner else 'SUCCEEDED') and safe['stop_confirmed']
        expected_kill=fault_owner and read('config.json')['fault_case']=='planner_exit'
        text=logs[i]+as2[i]
        expected_clean=10+int(fault_owner)-int(expected_kill)
        failures=re.findall(r'process has died \[pid (\d+), exit code (-?\d+)',text)
        expected=[]
        if expected_kill:
            event=read('mission-fault/result.json')['final_injector']['events'][0]
            expected=[(str(event['pid']),'-9')]
        checks[f'drone{i}_owned_nodes_exit_as_expected']=stops[i]['children']==dict(as2=0,ego=0) and text.count('process has finished cleanly')==expected_clean and failures==expected and not any(s in text for s in ('escalating to','Traceback'))
    checks['pc_thread_owned']=read('worker-identity.json')['thread']=='swarm-drone1' and read('worker-exit.json')['returncode']==0
    extra=stage=='fault' and read('config.json')['fault_vehicle']=='drone1'
    checks['pc_participants_unique']=sorted(r['participant'] for r in read('pc-runtime/dds/assignments.json'))==list(range(7,18 if extra else 17))
    camera=read('flight-camera/camera-result.json')
    rows=[json.loads(l) for l in (out/'flight-camera/camera.jsonl').read_text().splitlines()]
    metrics['camera_centroid_excursion_m']=max(math.dist(r['centroid_world_ned'],rows[0]['centroid_world_ned']) for r in rows)
    metrics['camera_actual_sample_span_s']=rows[-1]['wall']-rows[0]['wall']
    checks['live_moving_fleet_camera']=camera['passed'] and len(rows)>=100 and metrics['camera_centroid_excursion_m']>.5
    result=dict(passed=all(checks.values()),passed_checks=sum(checks.values()),total_checks=len(checks),checks=checks,metrics=metrics,
        scope='Mixed '+stage+' flight, native peer consumption, live moving camera and owned node cleanup')
    (out/'flight-session-audit.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':
    result=audit(Path(sys.argv[1]));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
