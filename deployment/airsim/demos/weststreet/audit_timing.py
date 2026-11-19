#!/usr/bin/env python3
"""Offline per-RGB-stamp pipeline timing; same Linux host monotonic clock only."""
import collections
import json
from pathlib import Path
import sqlite3
import statistics
import sys
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

root=Path(sys.argv[1]);events=[]
for path in(root/'flight-bag').glob('*.db3'):
 db=sqlite3.connect(path)
 for idx, in db.execute('select id from topics where name=?',('/diagnostics/vision_pipeline',)):
  for data, in db.execute('select data from messages where topic_id=? order by timestamp',(idx,)):
   events.append(json.loads(deserialize_message(data,String).data))
 db.close()
events.sort(key=lambda e:e['mono_ns'])
by_stage=collections.defaultdict(list);by_sample=collections.defaultdict(dict)
for e in events:
 by_stage[e['stage']].append(e)
 if e['sample_ns'] is not None:by_sample[e['sample_ns']].setdefault(e['stage'],e)

def stats(values):
 values=sorted(values)
 if not values:return dict(n=0)
 return dict(n=len(values),min_ms=values[0],p50_ms=statistics.median(values),
             p95_ms=values[min(len(values)-1,int((len(values)-1)*.95))],max_ms=values[-1])

stage_report={}
for stage,rows in by_stage.items():
 unique={e['sample_ns']:e for e in rows if e['sample_ns'] is not None}
 ordered=sorted(unique)
 arrivals=sorted(unique.values(),key=lambda e:e['mono_ns'])
 stage_report[stage]=dict(events=len(rows),unique_frames=len(unique),
   source_sample_gaps=stats([(b-a)/1e6 for a,b in zip(ordered,ordered[1:])]),
   monotonic_event_gaps=stats([(b['mono_ns']-a['mono_ns'])/1e6 for a,b in zip(arrivals,arrivals[1:])]))
pairs=[('sensor_receive','pose_queue'),('pose_queue','sensor_publish'),('sensor_receive','tracking_processed'),('sensor_receive','sensor_publish_begin'),('sensor_receive','sensor_publish'),
 ('sensor_publish_begin','detector_receive'),('sensor_publish','detector_receive'),
 ('detector_receive','buffer_put'),('buffer_put','buffer_take'),('buffer_take','infer_begin'),
 ('infer_begin','model_request'),('model_request','model_response'),
 ('projection_begin','projection_end'),('infer_begin','infer_end'),
 ('result_publish_begin','tracking_receive'),('result_publish','tracking_receive'),
 ('tracking_receive','tracking_take'),('tracking_take','tracking_processed')]
durations={}
for a,b in pairs:
 values=[];negative=0
 for sample,stages in by_sample.items():
  if a in stages and b in stages:
   dt=(stages[b]['mono_ns']-stages[a]['mono_ns'])/1e6
   if dt<0:negative+=1
   else:values.append(dt)
 durations[a+' -> '+b]=dict(**stats(values),negative_pairs=negative)
counts=collections.Counter((e.get('reason')or'unspecified')for e in events if e['stage'] in ('sensor_drop','detector_drop','tracking_reject','infer_error'))
seq_gaps={}
for node in set((e['node'],e['pid'])for e in events):
 seq=sorted(set(e['seq']for e in events if(e['node'],e['pid'])==node))
 seq_gaps[str(node)]=sum(max(0,b-a-1)for a,b in zip(seq,seq[1:]))
# Attribute the largest input-sample gap by following every upstream frame in it.
taken=sorted(by_stage.get('buffer_take',[]),key=lambda e:e['sample_ns'])
largest=None
if len(taken)>1:
 a,b=max(zip(taken,taken[1:]),key=lambda p:p[1]['sample_ns']-p[0]['sample_ns'])
 gap_samples=[]
 for sample,stages in sorted(by_sample.items()):
  if a['sample_ns']<sample<=b['sample_ns']:
   gap_samples.append(dict(sample_ns=sample,stages=list(stages),
       drop_reason=stages.get('sensor_drop',stages.get('detector_drop',{})).get('reason')))
 largest=dict(start_sample_ns=a['sample_ns'],end_sample_ns=b['sample_ns'],
   sample_gap_ms=(b['sample_ns']-a['sample_ns'])/1e6,
   take_interval_ms=(b['mono_ns']-a['mono_ns'])/1e6,upstream_frames=gap_samples)
report=dict(events=len(events),stage_counts=stage_report,durations=durations,
 drop_reasons=dict(counts),trace_sequence_missing=seq_gaps,largest_input_gap=largest,
 scope='Same-host monotonic durations; source sampling timestamps remain simulation time. First tap is native ROS image receive, NOT engine capture wall time. Missing initial/final or duplicate trace events do not prove drops.')
# Separate active-session samples from startup/landing tails.
active_durations={}
for a,b in pairs:
 values=[(stages[b]['mono_ns']-stages[a]['mono_ns'])/1e6
         for stages in by_sample.values() if a in stages and b in stages
         and 'tracking_processed' in stages and stages[b]['mono_ns']>=stages[a]['mono_ns']]
 active_durations[a+' -> '+b]=stats(values)
report['active_tracking_durations']=active_durations
report['buffer_overwrites']=sum(e.get('overwritten_ns') is not None for e in events if e['stage']=='buffer_put')
feedback=root/'mission/tracking-states.jsonl'
if feedback.exists() and events:
 states=[json.loads(line)for line in feedback.read_text().splitlines()]
 first=next((i for i,state in enumerate(states)if state['valid']),None)
 failure=next((state for state in states[first:]if not state['valid']),None) if first is not None else None
 if failure:
  start=round(failure['observation_stamp']*1e9)
  end=round(failure['stamp']*1e9)
  anchors=[e for e in events if e['stage']=='tracking_processed' and e['sample_ns']==start and e.get('observation_ns')==start]
  baseline=anchors[-1]['mono_ns'] if anchors else None
  offset=statistics.median(e['wall_ns']-e['mono_ns']for e in events)
  loss_mono=round(failure['wall_time']*1e9)-offset
  timeline=[]
  for e in events:
   if e['sample_ns'] is not None and start<=e['sample_ns']<=end and e['stage'] in ('sensor_publish','detector_receive','model_response','tracking_processed','sensor_drop'):
    timeline.append(dict(sample_ns=e['sample_ns'],stage=e['stage'],
       since_last_accepted_processing_ms=(e['mono_ns']-baseline)/1e6 if baseline is not None else None,
       filter_status=e.get('filter_status'),reason=e.get('reason')))
  report['first_loss']=dict(feedback=failure,timeline=timeline,
      since_last_accepted_processing_ms=(loss_mono-baseline)/1e6 if baseline is not None else None,
      wall_to_monotonic_mapping='same-host median offset; approximate feedback alignment')
(root/'timing-events.json').write_text(json.dumps(events,indent=2))
(root/'timing-report.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
if not events:raise SystemExit('No timing events; instrumentation not verified')
