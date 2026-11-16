"""Explicit flight orchestration on the accepted mixed compute topology."""
import importlib.util
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('hybrid_entry',HERE/'run.py')
hybrid=importlib.util.module_from_spec(spec);spec.loader.exec_module(hybrid)
single=hybrid.single

def clock_sample(config):
    before=time.time()
    remote=float(single.board_shell(config,"python3 -c 'import time;print(time.time())'",timeout=5))
    after=time.time()
    return dict(wall=after,rtt=after-before,offset=remote-(before+after)/2)

def flight(out,config):
    observer=json.loads((out/'observer.json').read_text())
    if not all(r['count']>=30 and r['receipt_age']<.75 and r['max_gap']<.75 for r in observer['topics'].values()):
        raise RuntimeError('Fresh ground observation required')
    clock_args=[sys.executable,'-B',hybrid.SINGLE/'check_clock.py',out,
                '--sync-margin',str(config.get('clock_sync_margin',.5))]
    if config.get('allow_clock_sync',False):clock_args.append('--sync-if-needed')
    single.execute(clock_args)
    camera_out=out/'flight-camera';camera_out.mkdir()
    for file in ('config.json','plan.json'):shutil.copyfile(out/file,camera_out/file)
    camera_log=(camera_out/'client.log').open('w')
    camera=subprocess.Popen([sys.executable,'-B',str(HERE/'camera.py'),str(camera_out),'900'],stdout=camera_log,stderr=subprocess.STDOUT)
    try:
        for stage in ('basic',config.get('mission_stage','short')):
            evidence=out/('mission-'+stage);evidence.mkdir()
            single.write(evidence/'deployment-config.json',config)
            if stage=='fault':shutil.copyfile(out/'mission-basic/stage-audit.json',evidence/'baseline-audit.json')
            states=[single.board_action(config,'check'),hybrid.pc_status(out)]
            if not all(s.get('ready') for s in states):raise RuntimeError('Both compute owners must be freshly ready before motion')
            rows=[]
            for i in (0,1):
                prefix=['docker','compose','-f',str(out/'compose.json'),'exec','-T','-w','/var/lib/flight_board',f'px4{i}']
                cmd=prefix+['/opt/px4-gazebo/bin/px4-commander','--instance',str(i)]
                status=single.execute(cmd+['status'])
                if not re.search(r'\bDisarmed\b',status):raise RuntimeError('PX4 not disarmed before Hold reset')
                rows.append(dict(vehicle=i,before=status,reset=single.execute(cmd+['mode','auto:loiter'])))
                deadline=time.monotonic()+10
                while True:
                    status=single.execute(cmd+['status'])
                    health=single.execute(prefix+['/opt/px4-gazebo/bin/px4-listener','--instance',str(i),'vehicle_status'])
                    if 'navigation mode: Hold' in status and 'pre_flight_checks_pass: True' in health:break
                    if time.monotonic()>deadline:raise RuntimeError('PX4 ground readiness failed')
                    time.sleep(.2)
            single.write(evidence/'ground-hold.json',rows)
            samples=[clock_sample(config) for _ in range(3)]
            single.write(evidence/'clock-before.json',samples)
            best=min(samples,key=lambda s:s['rtt'])
            if abs(best['offset'])+best['rtt']/2>.5:raise RuntimeError('Clock gate failed before motion')
            args=['docker','run','--rm','--init','--pull','never','--name','ego-hybrid-mission-'+out.name,
                '-p',hybrid.ports(config,config.get('mission_participant',18),config.get('mission_participant',18)),'--add-host','host.docker.internal:'+config['airsim_host'],
                '-e','ROS_DOMAIN_ID='+str(config['domain']),'-e','ROS_LOCALHOST_ONLY=0',
                '-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/session/mission.xml',
                '-e','OPENBLAS_NUM_THREADS=1','-e','PYTHONDONTWRITEBYTECODE=1',
                '-v',str(out)+':/session:ro','-v',str(evidence)+':/evidence',
                '-v',str(hybrid.DEMO/'pc')+':/input:ro','-v',str(HERE)+':/hybrid:ro',
                '-v',str(hybrid.SINGLE)+':/single:ro',
                '-v',str(hybrid.PROJECT/'src/tool/navigation/planner/ego_planner_v2/ros2_adapter/bridge')+':/adapter-source:ro',
                '-v',str(config['pc_workspace'])+':/work:ro']
            if stage!='basic':args+=['-v',str(out/'mission-basic')+':/baseline:ro']
            script={'crossing':'crossing_client.py','fault':'fault_client.py'}.get(stage,'flight_client.py')
            args+=['--entrypoint','bash',single.ONBOARD,'-c',hybrid.SETUP+'export PYTHONPATH=/adapter-source:$PYTHONPATH; exec python3 -B /hybrid/'+script+' '+stage]
            single.write(evidence/'deployment-session.json',dict(run_id=out.name,stage=stage,command=args,role='ROS client only'))
            print('Starting mixed flight stage: '+stage,flush=True)
            with (evidence/'client.log').open('w') as log:
                process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
                while process.poll() is None:
                    try:
                        sample=clock_sample(config)
                        with (evidence/'clock-offsets.jsonl').open('a') as stream:stream.write(json.dumps(sample)+'\n')
                        if abs(sample['offset'])+sample['rtt']/2>.5:raise RuntimeError('Runtime clock gate exceeded')
                        if camera.poll() is not None:raise RuntimeError('Camera observer exited early')
                    except Exception as exc:(evidence/'recovery.request').write_text(repr(exc))
                    time.sleep(2)
            single.write(evidence/'client-exit.json',dict(returncode=process.returncode))
            if process.returncode:raise RuntimeError('Mission failed; inspect recovery evidence before cleanup')
            # A basic-stage independent raw-log audit gates the short stage.
            auditor={'crossing':'audit_crossing.py','fault':'audit_fault.py'}.get(stage,'audit_flight.py')
            single.execute(['docker','run','--rm','--pull','never','--network','none',
                '-v',str(evidence)+':/evidence','-v',str(HERE)+':/hybrid:ro','-v',str(hybrid.SINGLE)+':/single:ro',
                '--entrypoint','python3',single.ONBOARD,'-B','/hybrid/'+auditor,'/evidence'])
    finally:
        (camera_out/'camera-stop').touch()
        camera.wait(timeout=15);camera_log.close()
        if camera.returncode:raise RuntimeError('Flight camera failed')

if __name__=='__main__':
    out=Path(sys.argv[1]).resolve();config=json.loads((out/'config.json').read_text())
    hybrid.mapped_profile(out/'mission.xml',config['pc_ip'],config['board_ip'],config['domain'],config.get('mission_participant',18))
    hybrid.ground(out,after_ground=flight)
