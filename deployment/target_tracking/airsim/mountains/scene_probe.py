"""Read scene identity, convoy metadata and real camera pixels before ROS startup."""
import argparse
import json
from pathlib import Path
import sys
import os
import shutil
import struct
import subprocess
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'simulator/airsim/sensors'))
from uav_as2_sensor_adapter.rpc import Rpc

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rpc=Rpc('127.0.0.1',49471,10)
    result={}
    try:
        result['vehicles']=rpc.call('listVehicles')
        assert result['vehicles']==['PX4'], 'Refuse camera API for unexpected vehicle configuration'
        result['convoy']=json.loads(rpc.call('simGetConvoyStatus'))
        result['vehicle_pose']=rpc.call('simGetVehiclePose','PX4')
        result['collision']=rpc.call('simGetCollisionInfo','PX4')
        result['all_objects']=rpc.call('simListSceneObjects','.*')
        result['nearby_geometry']={n:rpc.call('simGetObjectPose',n) for n in result['all_objects'] if any(k in n.lower() for k in ('truck','car','pickup','van','bridge'))}
        sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'object_detection/airsim'))
        from score_near import geopoint
        import numpy as np
        home=rpc.call('getHomeGeoPoint','PX4')
        result['vertical_clearance_rays']=[]
        for x,y in [(0,0),(-1,0),(1,0),(-2,0),(2,0),(0,-3),(0,3),(0,6)]:
            a0=np.array([x,y,-.3]);b=np.array([x,y,-4.])
            clear=rpc.call('simTestLineOfSightBetweenPoints',geopoint(a0,home),geopoint(b,home))
            result['vertical_clearance_rays'].append(dict(x=x,y=y,clear=clear))
        result['camera']=rpc.call('simGetCameraInfo','FrontCamera','PX4',False)
        result['objects']=rpc.call('simListSceneObjects','.*(Pickup|pickup|Van|Armored).*')
        result['object_poses']={n:rpc.call('simGetObjectPose',n) for n in result['objects']}
        images=rpc.call('simGetImages',[dict(camera_name='FrontCamera',image_type=0,pixels_as_float=False,compress=True)],'PX4',False)
        (a.output/'initial-camera.png').write_bytes(bytes(images[0]['image_data_uint8']))
        import cv2
        import numpy as np
        raw=rpc.call('simGetImages',[dict(camera_name='FrontCamera',image_type=0,
            pixels_as_float=False,compress=False)],'PX4',False)[0]
        pixels=np.frombuffer(bytes(raw['image_data_uint8']),dtype=np.uint8).reshape(raw['height'],raw['width'],3)
        # Compare static scene compressed and uncompressed captures for this binary.
        png=cv2.imdecode(np.frombuffer(bytes(images[0]['image_data_uint8']),np.uint8),cv2.IMREAD_COLOR)
        result['source_color_check']=dict(raw_vs_png_bgr_mae=float(np.abs(pixels.astype(float)-png).mean()),
            swapped_vs_png_bgr_mae=float(np.abs(pixels[:,:,::-1].astype(float)-png).mean()),
            limitation='consecutive captures, not atomic; static convoy before start')
        np.save(a.output/'initial-raw-scene.npy',pixels)
        if os.environ.get('DISPLAY') and shutil.which('xwd'):
            capture=subprocess.run(['xwd','-root','-silent','-out',str(a.output/'desktop.xwd')],timeout=5,capture_output=True)
            result['desktop_capture_exit']=capture.returncode
            if capture.returncode==0:
                from PIL import Image
                data=(a.output/'desktop.xwd').read_bytes();h=struct.unpack('>25I',data[:100])
                if h[11] in (24,32) and h[14:17]==(16711680,65280,255):
                    mode='BGR' if h[11]==24 else 'BGRX'
                    Image.frombytes('RGB',(h[4],h[5]),data[h[0]+h[19]*12:],'raw',mode,h[12],1).save(a.output/'desktop.png')
    finally:
        (a.output/'scene-reference.json').write_text(json.dumps(result,indent=2)+'\n');rpc.close()
if __name__=='__main__':main()
