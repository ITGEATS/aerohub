"""Use shared centroid camera with AirSim per-vehicle NED origins made explicit."""
import copy
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[5]
sys.path[:0]=[str(ROOT/'deployment/robopi/vendor'),str(Path(__file__).resolve().parent.parent/'pc')]
from airsim_rpc import Rpc

out=Path(sys.argv[1]);duration=float(sys.argv[2])
config=json.loads((out/'config.json').read_text())
plan=json.loads((out/'plan.json').read_text())
sys.path.insert(0,str(Path(config['aerohub_root'])/'deployment/swarm'))
from overhead_view import update_camera
from view import camera_options,downward_quaternion
rpc=Rpc(config.get('camera_host','127.0.0.1'),41451,3)
origins={v['airsim_name']:v['origin_ned_m'] for v in plan['topology']['vehicles']}
class WorldRpc:
    def call(self,method,*args):
        result=rpc.call(method,*args)
        if method=='simGetVehiclePose':
            result=copy.deepcopy(result)
            for axis,origin in zip(('x_val','y_val','z_val'),origins[args[0]]):result['position'][axis]+=origin
        return result
world=WorldRpc();options=camera_options()
settings=json.loads(rpc.call('getSettingsString'))
if 'swarm_overhead' not in settings.get('ExternalCameras',{}):raise RuntimeError('UE external camera not loaded')
(out/'actual-ue-settings.json').write_text(json.dumps(settings,indent=2))
before=rpc.call('simGetCameraInfo','swarm_overhead','',True)
perturbed=copy.deepcopy(before['pose']);perturbed['position']['x_val']+=5.
rpc.call('simSetCameraPose','swarm_overhead',perturbed,'',True)
perturbed_readback=rpc.call('simGetCameraInfo','swarm_overhead','',True)
rows=[];started=time.monotonic();deadline=started+duration
with (out/'camera.jsonl').open('w') as log:
    while time.monotonic()<deadline and not (out/'camera-stop').exists():
        start=time.monotonic()
        center=update_camera(world,list(origins),options)
        actual=rpc.call('simGetCameraInfo','swarm_overhead','',True)
        expected=[center[0],center[1],center[2]-options['altitude_m']]
        position=[actual['pose']['position'][k] for k in ('x_val','y_val','z_val')]
        q=actual['pose']['orientation'];wanted=downward_quaternion()
        qerror=min(math.dist([q[k] for k in wanted],[sign*wanted[k] for k in wanted]) for sign in (-1,1))
        row=dict(wall=time.time(),centroid_world_ned=center,expected=expected,actual=actual,
            position_error_m=math.dist(position,expected),orientation_error=qerror)
        rows.append(row);log.write(json.dumps(row)+'\n');log.flush()
        time.sleep(max(0,1/options['update_hz']-(time.monotonic()-start)))
image=rpc.call('simGetImage','swarm_overhead',0,'',True)
if isinstance(image,str):image=image.encode('latin1')
if image: (out/'overhead.png').write_bytes(bytes(image))
checks=dict(updated_continuously=len(rows)>=100,
    camera_perturbation_readback=abs(perturbed_readback['pose']['position']['x_val']-perturbed['position']['x_val'])<.05,
    follows_live_world_centroid=max(r['position_error_m'] for r in rows)<.05,
    downward=max(r['orientation_error'] for r in rows)<1e-4,
    rendered_png=bool(image) and bytes(image).startswith(b'\x89PNG'))
result=dict(passed=all(checks.values()),checks=checks,updates=len(rows),duration_s=time.monotonic()-started,requested_budget_s=duration,
    max_position_error_m=max(r['position_error_m'] for r in rows),origins=origins,
    before=before,perturbed=perturbed_readback,
    scope='Live-centroid feedback and camera-only perturbation recovery; movement coverage must be established from the recorded scene poses')
(out/'camera-result.json').write_text(json.dumps(result,indent=2))
raise SystemExit(0 if result['passed'] else 1)
