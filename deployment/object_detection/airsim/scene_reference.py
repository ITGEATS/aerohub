"""Test-only camera color/pose reference; runs before the shared bridge starts."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'deployment/robopi/vendor'),str(ROOT/'simulator/airsim/sensors')]
from uav_as2_sensor_adapter.rpc import Rpc
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
rpc=Rpc('127.0.0.1',49471,timeout=10.)
try:
    pose=rpc.call('simGetCameraInfo','FrontCamera','PX4',False)
    vehicle=rpc.call('simGetVehiclePose','PX4')
    home=rpc.call('getHomeGeoPoint','PX4')
    responses=rpc.call('simGetImages',[dict(camera_name='FrontCamera',image_type=0,pixels_as_float=False,compress=c) for c in (False,True)],'PX4',False)
    raw=np.frombuffer(bytes(responses[0]['image_data_uint8']),np.uint8).reshape(480,640,3)
    png=cv2.imdecode(np.frombuffer(bytes(responses[1]['image_data_uint8']),np.uint8),cv2.IMREAD_COLOR)
    direct=float(np.abs(raw.astype(float)-png).mean());swapped=float(np.abs(raw[:,:,::-1].astype(float)-png).mean())
    report=dict(camera=pose,vehicle=vehicle,home=home,raw_vs_png_bgr_mae=direct,swapped_vs_png_bgr_mae=swapped,
                stamps=[r['time_stamp'] for r in responses])
    (args.output/'scene-reference.json').write_text(json.dumps(report,indent=2)+'\n')
    cv2.imwrite(str(args.output/'scene-reference.png'),png)
    assert direct < swapped/2,'Raw camera color order differs from expected BGR; inspect actual binary'
finally:rpc.close()
