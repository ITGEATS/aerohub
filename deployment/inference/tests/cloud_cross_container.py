#!/usr/bin/env python3
"""Two-container Function/Tool test with synthetic RGB and a local provider.

Requires an already built ROS overlay and prepared /cloud-env. Owns and cleans
only its uniquely named internal Docker network/containers. No flight/API calls.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import uuid

ROOT = Path(__file__).resolve().parents[3]


def run(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', default='onboard_computer:v0.1.1')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    identifier = 'spf-cloud-test-' + uuid.uuid4().hex[:10]
    server = identifier + '-function'
    network = identifier
    common = ['--network', network, '-v', str(ROOT)+':/repo:ro',
              '-v', str(ROOT/'log/spf/inference-cloud')+':/evidence:ro',
              '-v', str(ROOT/'log/spf/pc-e2e/cloud-env')+':/cloud-env:ro',
              '-v', str(output)+':/cross', '-e', 'ROS_DOMAIN_ID=186',
              '-e', 'ROS_LOCALHOST_ONLY=0', '-e', 'PYTHONDONTWRITEBYTECODE=1',
              '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/repo/deployment/airsim/config/dds/fastdds_udp.xml',
              '--entrypoint', 'bash']
    setup = ('source /opt/ros/humble/setup.bash && '
             'source /evidence/ros_ws/install/setup.bash && '
             'export PYTHONPATH=/cloud-env/lib/python3.10/site-packages:/usr/lib/python3/dist-packages:$PYTHONPATH && ')
    run(['docker','network','create','--internal','--label','aerohub.test='+identifier,network], stdout=subprocess.DEVNULL)
    try:
        run(['docker','run','-d','--name',server,*common,args.image,'-lc',setup+
             'exec /cloud-env/bin/python /repo/deployment/inference/tests/cloud_function_fixture.py'], stdout=subprocess.DEVNULL)
        deadline = time.monotonic()+20
        while not (output/'ready.json').exists() and time.monotonic()<deadline:
            time.sleep(.1)
        if not (output/'ready.json').exists():
            raise RuntimeError('Function fixture readiness timeout')
        run(['docker','run','--rm',*common,args.image,'-lc',setup+
             'exec python3 /repo/deployment/inference/tests/cloud_tool_fixture.py'],timeout=30)
        result = json.loads((output/'client.json').read_text())
        if not result['ok']:
            raise RuntimeError('cross-container Function request failed')
        (output/'scope.json').write_text(json.dumps(dict(
            scope='synthetic two-container ROS/HTTP/Cloud protocol only',
            no_real_cloud=True,no_flight=True,image=args.image,server=server),indent=2)+'\n')
        print('PASS: Tool container -> Inference Function container -> local Cloud protocol -> actual SPF consumer')
    finally:
        try:
            run(['docker','stop','--time','10',server],stdout=subprocess.DEVNULL)
            logs=subprocess.check_output(['docker','logs',server],stderr=subprocess.STDOUT,text=True)
            (output/'server.log').write_text(logs)
        finally:
            subprocess.run(['docker','rm','-f',server],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            subprocess.run(['docker','network','rm',network],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)


if __name__=='__main__':
    main()
