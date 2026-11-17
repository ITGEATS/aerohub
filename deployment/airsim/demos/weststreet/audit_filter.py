#!/usr/bin/env python3
"""Offline proof of measurement-filter consumption; no truth accuracy claim."""
import collections
import json
import math
from pathlib import Path
import sqlite3
import statistics
import sys
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

root=Path(sys.argv[1]);rows=[]
for path in(root/'flight-bag').glob('*.db3'):
 db=sqlite3.connect(path)
 for idx, in db.execute('select id from topics where name=?',('/drone0/tools/target_tracking/measurement_filter',)):
  for data, in db.execute('select data from messages where topic_id=? order by timestamp',(idx,)):
   rows.append(json.loads(deserialize_message(data,String).data))
 db.close()
corrections=[math.dist(row['raw'][0],row['filtered'][0]) for row in rows
             if row['status']=='smoothed' and len(row['raw'])==len(row['filtered'])==1]
report=dict(samples=len(rows),statuses=dict(collections.Counter(row['status']for row in rows)),
 median_correction_m=statistics.median(corrections)if corrections else None,
 max_correction_m=max(corrections)if corrections else None,
 scope='Real raw/filtered pipeline consumption and correction, NOT accuracy vs truth')
(root/'filter-audit.json').write_text(json.dumps(report,indent=2))
(root/'filter-observations.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(report,indent=2))
