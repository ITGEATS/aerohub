"""Offline mixed ground audit; no flight or native MINCO trajectory claim."""
import hashlib
import json
from pathlib import Path
import sys
import tarfile

def audit(out):
    read=lambda name:json.loads((out/name).read_text())
    obs=read('observer.json');checks={};metrics={}
    checks['observer_40_seconds']=obs['duration']>=40
    for topic,row in obs['topics'].items():
        checks['fresh_unique:'+topic]=(row['count']>=30 and row['receipt_age']<.75 and row['max_gap']<.75
            and len(obs['graph'][topic]['publishers'])==1)
        checks['never_armed:'+topic]=not row['ever_armed']
    for ns in ('drone0','drone1'):
        state=obs['topics'][f'/{ns}/ego/status']['latest']
        checks[ns+'_waiting_owned_zero_goals']=(state['vehicles']==[ns] and state['interface_scope']==ns and state['counts']['goals']==0
            and obs['topics'][f'/{ns}/ego/status']['states']==['WAITING'])
        checks[ns+'_adapter_namespace']=obs['graph'][f'/{ns}/ego/status']['publishers'][0]['namespace']=='/'+ns
        checks[ns+'_no_goal_publisher']=not obs['graph'][f'/{ns}/ego/goal']['publishers']
        service=obs['services'][f'/{ns}/ego_as2_adapter/get_parameters']
        checks[ns+'_actual_parameter_service']=bool(service.get('response')) and service['response']['values'][0]['string_value']==ns and service['rtt']<5
        peer='drone1' if ns=='drone0' else 'drone0'
        checks[ns+'_adapter_peer_subscription']=any(p['node']=='ego_as2_adapter' and p['namespace']=='/'+ns
            for p in obs['graph'][f'/{peer}/navigation/odom']['subscriptions'])
        for suffix in ('navigation/points','ego/native/points'):checks[ns+'_'+suffix+'_nonempty']=obs['topics'][f'/{ns}/{suffix}'].get('points',0)>0
    checks['no_global_ego_endpoints']=not any(t.startswith('/ego/') for t,_ in obs['all_topics'])
    minco=obs['graph']['/ego_swarm/trajectory']
    for role in ('publishers','subscriptions'):
        checks['native_MINCO_'+role]=sorted((p['node'],p['namespace']) for p in minco[role])==[('ego_planner','/drone0'),('ego_planner','/drone1')]
    archive=max(out.glob('board-*.tar.gz'))
    checks['board_archive_hash']=hashlib.sha256(archive.read_bytes()).hexdigest()==json.loads(archive.with_suffix('.sha256.json').read_text())['sha256']
    with tarfile.open(archive) as tar:
        def load(name):return tar.extractfile(next(m for m in tar.getmembers() if m.name.removeprefix('./')==name)).read().decode()
        histories={'drone0':[json.loads(line) for line in load('status-history.jsonl').splitlines()]}
        stops={'drone0':json.loads(load('stopped.json'))}
        logs={'drone0':load('as2.log')+load('ego.log')}
    histories['drone1']=[json.loads(line) for line in (out/'pc-runtime/status-history.jsonl').read_text().splitlines()]
    stops['drone1']=read('pc-runtime/stopped.json')
    logs['drone1']=(out/'pc-runtime/as2.log').read_text()+(out/'pc-runtime/ego.log').read_text()
    for ns,rows in histories.items():
        start=None;longest=0
        for row in rows:
            probe=row.get('minco_type_probe',{})
            good=row['ready'] and row.get('peer_odom_fresh') and probe.get('fresh') and probe.get('received',0)>=20 and probe.get('invalid',1)==0
            if good:
                if start is None:start=row['written_at']
                longest=max(longest,row['written_at']-start)
            else:start=None
        metrics[ns+'_longest_joint_ready_s']=longest
        checks[ns+'_continuous_ready_30s']=longest>=30
        checks[ns+'_never_armed']=all(r.get('armed') is not True and r.get('px4_armed') is not True for r in rows)
        checks[ns+'_safe_stop']=stops[ns]['safe_state']['armed'] is False and stops[ns]['safe_state']['px4_armed'] is False and stops[ns]['safe_state']['behavior_idle']
        checks[ns+'_launch_exit_zero']=stops[ns]['children']==dict(as2=0,ego=0)
        checks[ns+'_all_ten_nodes_clean']=logs[ns].count('process has finished cleanly')==10 and not any(x in logs[ns] for x in ('escalating to','exit code -','Traceback'))
        metrics[ns+'_type_probe_received']=rows[-1]['minco_type_probe']['received']
    checks['pc_worker_thread_owned']=read('worker-identity.json')['thread']=='swarm-drone1' and read('worker-exit.json')['returncode']==0
    allocations=read('pc-runtime/dds/assignments.json')
    checks['ten_unique_pc_child_participants']=sorted(r['participant'] for r in allocations)==list(range(7,17))
    camera=read('camera-result.json')
    checks['live_shared_centroid_camera']=camera['passed'] and camera['updates']>=100
    metrics['camera_max_position_error_m']=camera['max_position_error_m']
    result=dict(passed=all(checks.values()),passed_checks=sum(checks.values()),total_checks=len(checks),checks=checks,metrics=metrics,
        scope='Two real disarmed PX4/AirSim UAVs; RoboPi drone0 and PC drone1 compute; no flight goals',
        limitations=['MINCO graph and isolated bidirectional message-type probes are verified; actual native trajectory exchange requires a planning/flight case.',
                     'Dynamic overhead camera acceptance is recorded separately and required for the complete ground result.'])
    (out/'ground-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    result=audit(Path(sys.argv[1]));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
