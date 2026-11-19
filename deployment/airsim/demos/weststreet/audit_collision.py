#!/usr/bin/env python3
"""Offline: airborne UE contacts aligned with recorded own/reference trajectories."""
import argparse
import datetime
import json
from pathlib import Path
import re
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir',type=Path)
    args=parser.parse_args();r=args.run_dir
    events=json.loads((r/'mission/events.json').read_text())
    begin=next(e['wall_time']for e in events if e['event']=='takeoff')
    end=next((e['wall_time']for e in events if e['event']=='land'),float('inf'))
    contacts=[]
    pattern=r"\[(\d{4}\.\d\d\.\d\d-\d\d\.\d\d\.\d\d:\d{3})\].*Collision detected between '([^']+)' and '([^']+)' at z= '([^']+)'"
    for match in re.finditer(pattern,(r/'simulator/projectairsim.log').read_text(errors='replace')):
        wall=datetime.datetime.strptime(match[1],'%Y.%m.%d-%H.%M.%S:%f').replace(tzinfo=datetime.timezone.utc).timestamp()
        if begin<=wall<=end:
            contacts.append(dict(wall=wall,actor=match[2],other=match[3],ue_z_cm=float(match[4])))
    odom=[];references=[]
    for path in (r/'flight-bag').glob('*.db3'):
        db=sqlite3.connect(path);topics={i:(name,typ)for i,name,typ in db.execute('select id,name,type from topics')}
        for tid,wall,data in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
            name,typ=topics[tid]
            if name not in ('/drone0/sensor_measurements/odom','/drone0/motion_reference/trajectory'):continue
            msg=deserialize_message(data,get_message(typ))
            if name.endswith('/odom'):
                p=msg.pose.pose.position;v=msg.twist.twist.linear
                odom.append(dict(wall=wall/1e9,position=[p.x,p.y,p.z],velocity=[v.x,v.y,v.z]))
            elif msg.setpoints:
                p=msg.setpoints[0].position
                references.append(dict(wall=wall/1e9,position=[p.x,p.y,p.z]))
        db.close()
    if contacts:
        first=contacts[0]
        first['odom']=min(odom,key=lambda x:abs(x['wall']-first['wall'])) if odom else None
        first['reference']=min(references,key=lambda x:abs(x['wall']-first['wall'])) if references else None
        first['seconds_after_takeoff']=first['wall']-begin
    else:first=None
    report=dict(airborne_contacts=len(contacts),first_contact=first,passed=not contacts,
                scope='UE contact log vs own/reference; not a full contact sensor audit')
    (r/'collision-audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    if contacts:raise SystemExit(1)

if __name__=='__main__':main()
