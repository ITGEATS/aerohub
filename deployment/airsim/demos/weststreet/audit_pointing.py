#!/usr/bin/env python3
"""Offline camera command and detection-centering evidence; no control/truth API."""
import json
from pathlib import Path
import sqlite3
import statistics
import sys
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

root=Path(sys.argv[1]);rows=[]
for path in(root/'flight-bag').glob('*.db3'):
 db=sqlite3.connect(path)
 for idx, in db.execute('select id from topics where name=?',('/drone0/gimbal/status',)):
  for data, in db.execute('select data from messages where topic_id=? order by timestamp',(idx,)):
   rows.append(json.loads(deserialize_message(data,String).data))
 db.close()
xs=[];ys=[];frames=0;empty=0
for line in(root/'detections/calls.jsonl').read_text().splitlines():
 record=json.loads(line);frames+=1
 detections=record.get('detections',[])
 if not detections:empty+=1;continue
 box=max(detections,key=lambda d:d['confidence'])['xyxy']
 meta=json.loads((root/'detections'/(str(record['stamp_ns'])+'.json')).read_text())
 xs.append(abs((box[0]+box[2])/2-meta['width']/2)/meta['width'])
 ys.append(abs((box[1]+box[3])/2-meta['height']/2)/meta['height'])
report=dict(confirmed_pose_changes=max((x['confirmed_changes']for x in rows),default=0),
 rpc_errors=sorted(set(x['error']for x in rows if x['error'])),
 applied_pitch_range=[min(x['applied_pitch']for x in rows),max(x['applied_pitch']for x in rows)]if rows else None,
 applied_yaw_range=[min(x['applied_yaw']for x in rows),max(x['applied_yaw']for x in rows)]if rows else None,
 rgb_requests=frames,empty_detections=empty,
 median_horizontal_center_error_fraction=statistics.median(xs)if xs else None,
 median_vertical_center_error_fraction=statistics.median(ys)if ys else None,
 scope='actual RPC confirmations and highest-confidence visual box centering, not Actor truth or full pursuit acceptance')
(root/'pointing-audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
