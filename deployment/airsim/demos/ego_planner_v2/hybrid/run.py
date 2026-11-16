"""Mixed RoboPi/PC ground acceptance. No arm, takeoff or planner-goal operation."""
import argparse
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time

HERE=Path(__file__).resolve().parent
PROJECT=HERE.parents[4]
DEMO=HERE.parent
SINGLE=DEMO/'robopi'
sys.path.insert(0,str(SINGLE))
spec=importlib.util.spec_from_file_location('ego_single_entry',SINGLE/'run.py')
single=importlib.util.module_from_spec(spec);spec.loader.exec_module(single)
from dds_mapped import mapped_profile

SETUP='set -e; '+''.join('source '+p+'/local_setup.bash; ' for p in (
    '/opt/ros/humble','/opt/px4_ws/install','/work/install/humble','/root/aerostack2_ws/install',
    '/work/ground_overlay/install','/work/ego_runtime/install'))
write=single.write



def normalize_config(config):
    """Validate the supported topology before creating files or touching devices."""
    config=dict(config)
    if config.get('vehicle') != 'drone0':
        raise ValueError('Mixed deployment requires the per-vehicle Drone0 board interface')
    stage=config.setdefault('mission_stage','short')
    if stage not in ('short','crossing','fault'):
        raise ValueError('Supported stages: short, crossing, fault')
    if stage=='fault':
        if config.get('fault_case') not in ('cancel','planner_exit','cloud','odom') or config.get('fault_vehicle') not in ('drone0','drone1'):
            raise ValueError('Fault stage requires explicit fault_case and fault_vehicle')
    elif config.get('fault_case') or config.get('fault_vehicle'):
        raise ValueError('Fault configuration requires mission_stage=fault')
    participant=config.setdefault('mission_participant',18)
    if type(participant) is not int or not 18<=participant<=31:
        raise ValueError('Mission participant must be 18..31, outside compute IDs 6..17')
    if type(config.setdefault('allow_clock_sync',False)) is not bool:
        raise ValueError('allow_clock_sync must be an explicit boolean')
    margin=config.setdefault('clock_sync_margin',.5)
    if isinstance(margin,bool) or not isinstance(margin,(int,float)) or not 0<margin<=.5:
        raise ValueError('clock_sync_margin must be in (0, 0.5]')
    value=config.get('pc_workspace') or os.environ.get('EGO_PC_WORK')
    if not value or not Path(value).is_absolute():
        raise ValueError('Set an absolute pc_workspace or EGO_PC_WORK')
    work=Path(value).resolve()
    if work==PROJECT or PROJECT in work.parents:
        raise ValueError('Use an external EGO workspace')
    if (work/'component').read_text().strip()!='ego-planner-v2':
        raise ValueError('Workspace belongs to another component')
    manifest=json.loads((work/'source-manifest.json').read_text())
    for rel in ('src/tool/navigation/planner/ego_planner_v2/ros2_adapter/native',
                'src/tool/navigation/planner/ego_planner_v2/ros2_adapter/bridge',
                'src/tool/navigation/interfaces','src/tool/navigation/framework','src/behavior/follow_path'):
        for path in (PROJECT/rel).rglob('*'):
            if not path.is_file() or '__pycache__' in path.parts:continue
            key=path.relative_to(PROJECT).as_posix()
            if manifest.get(key)!=hashlib.sha256(path.read_bytes()).hexdigest():
                raise ValueError('Rebuild workspace from current sources: '+key)
    config.update(pc_workspace=str(work),aerohub_root=str(PROJECT))
    return config


def ports(config,first,last):
    base=7400+250*config['domain']+10
    return f"{config['pc_ip']}:{base+2*first}-{base+2*last+1}:{base+2*first}-{base+2*last+1}/udp"


def compute_command(out,config):
    return ['docker','run','--init','--pull','never','--name','ego-hybrid-compute-'+out.name,
        '--memory','3g','--cpus','4','-p',ports(config,6,17 if config.get('fault_case') else 16),
        '-v',str(out)+':/session:ro',
        '-v',str(SINGLE)+':/single:ro','-v',str(HERE)+':/hybrid:ro',
        '-v',str(PROJECT/'src/tool/navigation/planner/ego_planner_v2/ros2_adapter/bridge')+':/adapter-source:ro',
        '-v',str(config['pc_workspace'])+':/work:ro',
        '-e','ROS_DOMAIN_ID='+str(config['domain']),'-e','ROS_LOCALHOST_ONLY=0',
        '-e','PYTHONDONTWRITEBYTECODE=1','-e','OPENBLAS_NUM_THREADS=1',
        '-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
        '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/session/drone1/dds.xml',
        '-e','EGO_SESSION_ROOT=/session/drone1','--entrypoint','bash',single.ONBOARD,'-c',SETUP+
        'export PYTHONPATH=/adapter-source:$PYTHONPATH; mkdir -p /runtime; exec python3 -B /single/runtime.py /runtime']


def prepare(out,config):
    config=normalize_config(config)
    single.validate(config)
    if config.get('vehicle')!='drone0' or config['dds_transport']!='udp_mapped':raise ValueError('Scoped mapped-UDP candidate required')
    single.run_id(out.name)
    out.mkdir(parents=True,exist_ok=False)
    config=dict(config,run_id=out.name,hybrid=True,remote=config['candidate']+'/sessions/'+out.name)
    write(out/'config.json',config)
    (out/'pc-runtime').mkdir()
    schema=dict(schema_version=1,deployment={'stage':'hybrid_board'},fleet=dict(vehicle_count=2,board_vehicle_index=0,name_prefix='drone',
        airsim_name_prefix='Drone',initial_spacing_m=4.),
        nodes={'board':{'launch_command':['python3',config['remote']+'/board.py','start','--run-id',out.name]},
               'onboard_computer':{'image':single.ONBOARD,'launch_command':compute_command(out,config)}},
        coordination={'mode':'decentralized'},ground_station={'required':True,'placement':'pc',
            'launch_command':['python3','/hybrid/observe.py','/session','40']},
        airsim=dict(base_settings='settings.snapshot.json',rpc_port=41451,
            ports=dict(tcp_base=4560,control_local_base=14540,control_remote_base=14580,px4_instance_base=0,px4_system_id_base=135,xrce_agent_base=8888),
            camera=dict(name='swarm_overhead',altitude_m=25,fov_degrees=90,update_hz=5)),
        validation={'startup_timeout_s':120,'smoke_duration_s':40})
    write(out/'swarm-config.json',schema)
    single.execute(['docker','run','--rm','--network','none','--pull','never',
        '-v',str(config['aerohub_root'])+':/aerohub:ro','-v',str(HERE)+':/hybrid:ro',
        '-v',str(DEMO/'pc/settings.snapshot.json')+':/snapshot.json:ro',
        '-v',str(out)+':/session','--entrypoint','python3',single.ONBOARD,'-B','/hybrid/hybrid_plan.py',
        '--config','/session/swarm-config.json','--snapshot','/snapshot.json','--shared-root','/aerohub','--output','/session/plan.json'])
    plan=json.loads((out/'plan.json').read_text())
    plan['ego']['task']['task_id']='ego_'+out.name
    for vehicle in ('drone0','drone1'):
        directory=out/('bundle' if vehicle=='drone0' else 'drone1');directory.mkdir()
        local=dict(config,vehicle=vehicle)
        if config.get('fault_case') and vehicle!=config.get('fault_vehicle'):
            local.pop('fault_case')
        if vehicle=='drone1':local.update(dds_launch_prefix='/hybrid/dds_exec.py',ego_launch_file='/adapter-source/launch/ego_as2.launch.py')
        write(directory/'config.json',local)
        for file in ('runtime.py','board.py','policy.py','interfaces.py','ground.launch.py','as2.yaml','controller.yaml'):
            shutil.copyfile(SINGLE/file,directory/file)
        if local.get('fault_case'):
            for source,target in [('fault_launch.py','fault.launch.py'),('fault_injector.py','fault_injector.py')]:
                shutil.copyfile(HERE/source,directory/target)
        vehicle_ego=dict(plan['ego'])
        if config.get('mission_stage')=='crossing':
            shutil.copyfile(DEMO/'pc/ego_crossing_native.yaml',directory/'native-crossing.yaml')
            vehicle_ego['native_parameters_file']=(config['remote'] if vehicle=='drone0' else '/session/drone1')+'/native-crossing.yaml'
        write(directory/'ego.json',vehicle_ego)
        write(directory/'fleet.json',{'vehicles':{vehicle:dict(fmu_prefix='/'+vehicle,target_system_id=135+int(vehicle[-1]))}})
        if vehicle=='drone0':single.dds(directory/'dds.xml',config['board_ip'],config['pc_ip'],config['domain'])
        else:mapped_profile(directory/'dds.xml',config['pc_ip'],config['board_ip'],config['domain'],6)
    # Current UE settings remain intact; the generated camera settings require
    # explicit runtime verification rather than claiming that the file is active.
    write(out/'settings.json',plan['settings'])
    services={}
    for i in (0,1):
        directory=out/f'flight{i}';directory.mkdir()
        for role,index in [('agent',2*i),('sensors',2*i+1)]:
            mapped_profile(directory/f'pc-dds-{role}.xml',config['pc_ip'],config['board_ip'],config['domain'],index,
                profile_name='px4_participant' if role=='agent' else 'ego_mapped_udp')
        shutil.copyfile(SINGLE/'pc_supervisor.py',directory/'pc_supervisor.py')
        env=dict(ROS_DOMAIN_ID=str(config['domain']),ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
            AIRSIM_HOST=config['airsim_host'],AIRSIM_RPC_PORT='41451',VEHICLE_NAME=f'Drone{i}',VEHICLE_NAMESPACE=f'drone{i}',
            PX4_INSTANCE=str(i),PX4_SYSTEM_ID=str(135+i),XRCE_AGENT_PORT=str(8888+i),FLIGHT_BOARD_CAMERAS='0')
        services[f'px4{i}']=dict(image=single.FLIGHT,pull_policy='never',init=True,environment=env,
            volumes=[str(directory)+':/session:ro',str(out/'settings.json')+':/config/settings.json:ro'],
            ports=[ports(config,2*i,2*i+1),f'127.0.0.1:{14580+i}:{14580+i}/udp'],
            entrypoint=['bash','-c','set -e; source /opt/ros/humble/setup.bash; source /opt/platform_sensors/install/setup.bash; exec python3 /session/pc_supervisor.py "$@"','ego-flight'],command=['vehicle'])
    mapped_profile(out/'flight0/pc-dds-clock.xml',config['pc_ip'],config['board_ip'],config['domain'],4)
    services['clock']=dict(services['px40'],ports=[ports(config,4,4)],command=['clock'])
    write(out/'compose.json',{'name':'ego-hybrid-'+out.name,'services':services})
    mapped_profile(out/'observer.xml',config['pc_ip'],config['board_ip'],config['domain'],5)
    write(out/'sources.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (out/'bundle').iterdir()})
    print(out)


class ComputeWorker(threading.Thread):
    """One foreground container owned by one PC UAV supervisor thread."""
    def __init__(self,out,config):
        super().__init__(name='swarm-drone1',daemon=False)
        self.out,self.config=out,config
        self.process=None;self.error=None
    def run(self):
        self.thread_id=threading.get_ident()
        try:
            with (self.out/'pc-worker.log').open('w') as log:
                self.process=subprocess.Popen(compute_command(self.out,self.config),stdout=log,stderr=subprocess.STDOUT)
                write(self.out/'worker-identity.json',dict(thread=self.name,thread_id=self.thread_id,pid=self.process.pid,
                    vehicle='drone1',placement='pc',command=compute_command(self.out,self.config)))
                self.process.wait()
                write(self.out/'worker-exit.json',dict(returncode=self.process.returncode))
        except Exception as exc:self.error=repr(exc)


def pc_status(out):
    try:
        result=subprocess.run(['docker','exec','ego-hybrid-compute-'+out.name,'cat','/runtime/status.json'],
            capture_output=True,text=True,timeout=5)
    except subprocess.TimeoutExpired as exc:
        with (out/'status-query-errors.jsonl').open('a') as log:
            log.write(json.dumps(dict(wall=time.time(),error=str(exc)))+'\n')
        return {}  # Unknown is unready; the original startup deadline still applies.
    return json.loads(result.stdout) if result.returncode==0 else {}


def ground(out, after_ground=None):
    config=json.loads((out/'config.json').read_text())
    single.validate(config)
    # Reuse manifest, HDC hash checks and board lifecycle; no candidate mutation.
    single.execute([sys.executable,'-B',SINGLE/'run.py','deploy','--output',out])
    compose=['docker','compose','-f',str(out/'compose.json')]
    single.execute(compose+['up','-d','--no-build'])
    write(out/'board-start.json',single.board_action(config,'start'))
    worker=ComputeWorker(out,config);worker.start()
    deadline=time.monotonic()+120
    error=None
    try:
        while True:
            board=single.board_action(config,'status');pc=pc_status(out)
            if worker.error or worker.process and worker.process.poll() is not None:raise RuntimeError('PC compute exited: '+str(worker.error))
            if board['ready'] and pc.get('ready') and time.time()-pc.get('written_at',0)<2 and all(s.get('peer_odom_fresh') and s.get('minco_type_probe',{}).get('received',0)>=20 for s in (board['status'],pc)):
                write(out/'both-ready.json',dict(board=board,pc=pc));break
            if time.monotonic()>deadline:raise RuntimeError('Mixed readiness timeout; inspect both runtime logs')
            time.sleep(1)
        print('Both compute nodes ready; starting independent 40-second observer',flush=True)
        camera_log=(out/'camera.log').open('w')
        camera=subprocess.Popen([sys.executable,'-B',str(HERE/'camera.py'),str(out),'45'],stdout=camera_log,stderr=subprocess.STDOUT)
        single.execute(['docker','run','--rm','--init','--pull','never','--name','ego-hybrid-observer-'+out.name,
            '-p',ports(config,5,5),'-v',str(out)+':/session','-v',str(HERE)+':/hybrid:ro',
            '-v',str(config['pc_workspace'])+':/work:ro','-e','ROS_DOMAIN_ID='+str(config['domain']),
            '-e','ROS_LOCALHOST_ONLY=0','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/session/observer.xml',
            '--entrypoint','bash',single.ONBOARD,'-c',SETUP+'exec python3 -B /hybrid/observe.py /session 40'],timeout=70)
        camera.wait(timeout=20);camera_log.close()
        if camera.returncode:raise RuntimeError('Dynamic camera verification failed')
        if after_ground is not None:after_ground(out,config)
    except Exception as exc:
        error=repr(exc);write(out/'failure.json',dict(error=error))
    # Each runtime independently requires fresh disarmed/idle feedback before
    # stopping. No unconditional Docker kill or shutdown of the flight clocks.
    write(out/'board-stop.json',single.board_action(config,'stop'))
    single.execute(['docker','exec','ego-hybrid-compute-'+out.name,'touch','/runtime/stop.request'])
    worker.join(timeout=30)
    if worker.is_alive():raise RuntimeError('PC stop not confirmed; retain PX4 and clock')
    single.execute(['docker','cp','ego-hybrid-compute-'+out.name+':/runtime/.',str(out/'pc-runtime')])
    if not (out/'pc-runtime/stopped.json').exists():raise RuntimeError('Missing PC safe stop; retain PX4 and clock')
    single.execute([sys.executable,'-B',SINGLE/'run.py','collect','--output',out])
    (out/'pc.log').write_text(single.execute(compose+['logs','--no-color']))
    single.execute(compose+['down'])
    single.execute(['docker','rm','ego-hybrid-compute-'+out.name])
    if error:raise RuntimeError(error)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['prepare','ground'])
    p.add_argument('--config',type=Path);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.action=='prepare':prepare(args.output.resolve(),json.loads(args.config.read_text(encoding='utf-8-sig')))
    else:ground(args.output.resolve())
