"""Fixed-image, independent client/adapter DDS check against an existing GPU worker."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import uuid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--endpoint', default='http://127.0.0.1:18083')
    parser.add_argument('--output', type=Path, default=Path('log/grounding-dino-inference'))
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[3]
    image = args.image.resolve().relative_to(repo)
    output = args.output.resolve().relative_to(repo)
    token = 'grounding-dino-check-'+uuid.uuid4().hex[:8]
    containers = []
    identities = {}
    def run(argv, **kwargs):
        return subprocess.run(argv, check=True, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, **kwargs).stdout.strip()
    source = ('source /opt/ros/humble/setup.bash && '
        'source /workspace/log/grounding-dino-inference/ros_ws/install/setup.bash && ')
    try:
        for role, fixed_image in [('server','onboard_computer:v0.1.0'),
                                  ('client','osrf/ros:humble-desktop-full-jammy')]:
            identities[role] = dict(image=fixed_image, image_id=run(['docker','image','inspect',fixed_image,'--format','{{.Id}}']))
            command = source
            if role == 'server':
                command += ('exec ros2 run uav_inference grounding_dino_node --ros-args '
                    '-r __ns:=/probe -p endpoint:='+shlex.quote(args.endpoint)+' -p timeout_s:=10.0')
            else:
                command += ('exec python3 /workspace/deployment/inference/tests/grounding_dino_ros_probe.py '
                    '--image '+shlex.quote('/workspace/'+str(image))+
                    ' --baseline '+shlex.quote('/workspace/'+str(output/'baseline.json'))+
                    ' --output '+shlex.quote('/workspace/'+str(output/'ros-result.json')))
            identifier = run(['docker','run','-d','--init','--name',token+'-'+role,
                '--network','host','--entrypoint','bash','-e','ROS_DOMAIN_ID=227',
                '-e','ROS_LOCALHOST_ONLY=0','-e','PYTHONDONTWRITEBYTECODE=1',
                '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/deployment/airsim/config/dds/fastdds_udp.xml',
                '-v',str(repo)+':/workspace','-w','/tmp',fixed_image,'-lc',command])
            containers.append((role,identifier))
        result = run(['docker','wait',containers[-1][1]],timeout=150)
        if result != '0':
            raise RuntimeError('client failed: '+run(['docker','logs',containers[-1][1]]))
        (repo/output/'ros-topology.json').write_text(json.dumps(dict(identities=identities,
            endpoint=args.endpoint, domain=227, scope='two independent fixed-image containers, host DDS to CUDA worker',
            containers=containers),indent=2)+'\n')
    finally:
        errors = []
        for role, identifier in reversed(containers):
            logs = subprocess.run(['docker','logs',identifier],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            (repo/output/('ros-'+role+'.log')).write_bytes(logs.stdout)
            stopped = subprocess.run(['docker','stop','--time','10',identifier],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=20)
            removed = subprocess.run(['docker','rm','-f',identifier],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=10)
            if stopped.returncode or removed.returncode:
                errors.append(dict(role=role,stop=stopped.returncode,remove=removed.returncode))
        (repo/output/'ros-cleanup.json').write_text(json.dumps(dict(success=not errors,
            owned_containers=containers,errors=errors),indent=2)+'\n')
        if errors:
            raise RuntimeError('cleanup failed: '+str(errors))


if __name__ == '__main__':
    main()
