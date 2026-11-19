#!/usr/bin/env python3
"""Offline native configuration/capture TF audit; not a render-pose oracle."""
import bisect
import json
import math
from pathlib import Path
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'simulator/projectairsim'))
from gimbal import interpolate
from uav_as2_sensor_adapter.conversion import multiply

root=Path(sys.argv[1]);body=[];camera=[];static_camera=False
for path in(root/'flight-bag').glob('*.db3'):
 db=sqlite3.connect(path)
 topics={i:n for i,n in db.execute('select id,name from topics')}
 for idx,data in db.execute('select topic_id,data from messages'):
  if topics[idx] not in('/tf','/tf_static'):continue
  for tf in deserialize_message(data,TFMessage).transforms:
   stamp=tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
   q=tf.transform.rotation;q=(q.x,q.y,q.z,q.w)
   if tf.child_frame_id=='drone0/base_link' and tf.header.frame_id=='drone0/odom':body.append((stamp,q))
   if tf.child_frame_id=='drone0/FrontCamera_optical':
    if topics[idx]=='/tf_static':static_camera=True
    else:camera.append((stamp,q))
 db.close()
body=sorted(dict(body).items());camera=sorted(dict(camera).items());stamps=[s for s,_ in body]
errors=[];tilts=[]
for stamp,qcam in camera:
 i=bisect.bisect_left(stamps,stamp)
 if not 0<i<len(body):continue
 a,b=body[i-1],body[i]
 if b[0]-a[0]>.1:continue
 q=interpolate(a[1],b[1],(stamp-a[0])/(b[0]-a[0]));world=multiply(q,qcam)
 inv=(-world[0],-world[1],-world[2],world[3])
 forward=multiply(multiply(world,(0.,0.,1.,0.)),inv)
 right=multiply(multiply(world,(1.,0.,0.,0.)),inv)
 errors.append(max(abs(math.degrees(math.asin(max(-1,min(1,v)))))for v in(forward[2],right[2])))
 up=multiply(multiply(q,(0.,0.,1.,0.)),(-q[0],-q[1],-q[2],q[3]))
 tilts.append(math.degrees(math.acos(max(-1,min(1,up[2])))))
scene=json.loads((root/'simulator/resolved_scene.json').read_text());active=[]
for actor in scene['actors']:
 for sensor in actor.get('robot-config',{}).get('sensors',[]):
  if sensor.get('type')=='camera' and sensor.get('enabled',True):
   for capture in sensor.get('capture-settings',[]):
    if capture.get('capture-enabled'):active.append(dict(camera=sensor['id'],image_type=capture['image-type'],interval=sensor.get('capture-interval'),gimbal=sensor.get('gimbal')))
intervals=[b[0]-a[0]for a,b in zip(camera,camera[1:])]
report=dict(active_captures=active,camera_in_static_tf=static_camera,camera_dynamic_samples=len(camera),
 unique_capture_hz=(len(camera)-1)/(camera[-1][0]-camera[0][0]) if len(camera)>1 else None,
 minimum_capture_interval_s=min(intervals)if intervals else None,
 matched_orientation_samples=len(errors),max_world_level_residual_deg=max(errors)if errors else None,
 max_vehicle_tilt_deg=max(tilts)if tilts else None,
 scope='Native spring-arm config + image-stamp TF vs independent PX4/AS2 body attitude; not atomically sampled renderer pose')
(root/'gimbal-audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
