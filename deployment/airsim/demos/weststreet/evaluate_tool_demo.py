#!/usr/bin/env python3
"""Offline ROS-bag audit. Never supplies truth or commands to the mission."""
import argparse
import json
import math
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir',type=Path)
    args = parser.parse_args()
    root = args.run_dir
    manifest=json.loads((root/'predeclared.json').read_text())
    result = json.loads((root/'mission/result.json').read_text())
    cleanup = json.loads((root/'cleanup.json').read_text())
    targets_file = root/'mission/targets.jsonl'
    targets = [json.loads(line) for line in targets_file.read_text().splitlines()] if targets_file.exists() else []
    samples = []
    references = []
    counts = {}
    for database in (root/'flight-bag').glob('*.db3'):
        db = sqlite3.connect(database)
        topics = {row[0]:(row[1],row[2]) for row in db.execute('select id,name,type from topics')}
        for topic_id, data in db.execute('select topic_id,data from messages order by timestamp'):
            topic, kind = topics[topic_id]
            counts[topic] = counts.get(topic,0)+1
            if topic not in ('/drone0/sensor_measurements/odom','/drone0/motion_reference/trajectory'):
                continue
            msg = deserialize_message(data,get_message(kind))
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if topic.endswith('/odom'):
                p, v = msg.pose.pose.position,msg.twist.twist.linear
                samples.append(dict(stamp=stamp,position=[p.x,p.y,p.z],speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)))
            else:
                references.append(stamp)
        db.close()
    motion = None
    if samples:
        samples.sort(key=lambda s:s['stamp'])
        ground = samples[0]['position'][2]
        motion = dict(sample_count=len(samples),first_position=samples[0]['position'],
            last_position=samples[-1]['position'],relative_height_max_m=max(s['position'][2]-ground for s in samples),
            max_speed_mps=max(s['speed'] for s in samples))
        if targets:
            begin, end = targets[0]['sim_time'],targets[-1]['sim_time']+.3
            pursuit = [s for s in samples if begin<=s['stamp']<=end]
            if pursuit:
                start = pursuit[0]['position']
                motion['pursuit_horizontal_displacement_m'] = max(math.dist(s['position'][:2],start[:2]) for s in pursuit)
                motion['pursuit_samples'] = len(pursuit)
                motion['actual_trajectory_references'] = sum(begin<=stamp<=end for stamp in references)
    gates = dict(mission_success=bool(result['success']),physical_disarm=bool(cleanup['disarm_confirmed']),
                 execution_cleanup=bool(cleanup['services_stopped'] or
                    (manifest.get('keep_scene',False) and cleanup.get('scene_retained',False) and
                     cleanup.get('execution_services_stopped',False) and cleanup['disarm_confirmed'])))
    if targets_file.exists():
        gates.update(runner_started=(root/'runner-start.json').exists(),
            actual_navigation_displacement=motion is not None and motion.get('pursuit_horizontal_displacement_m',0)>=1.,
            execution_references=motion is not None and motion.get('actual_trajectory_references',0)>0)
    report = dict(scope='bounded Tool-demo phase; not complete reproduction',
        cleanup_mode='retained_disarmed_scene' if manifest.get('keep_scene',False) else 'all_services_stopped',gates=gates,
        phase_passed=all(gates.values()),motion=motion,topic_counts=counts,target_updates=len(targets),
        unique_capture_ids=len(set(row['capture_id'] for row in targets)),
        not_covered=['full loop','target-yaw','comprehensive collision','reliability','full fault matrix'])
    (root/'evaluation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    if not report['phase_passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
