#!/usr/bin/env python3
"""Offline native/ROS frame rates and per-socket drops; recorded CDR String only."""
import json,sqlite3,struct,collections,statistics,pathlib,sys
r=pathlib.Path(sys.argv[1]); ev=[];udp=[]
for p in (r/'flight-bag').glob('*.db3'):
 db=sqlite3.connect(p)
 for ident,name in db.execute("select id,name from topics where name in ('/diagnostics/vision_pipeline','/diagnostics/vision_udp')"):
  for data, in db.execute('select data from messages where topic_id=?',(ident,)):
   assert data[:2]==b'\x00\x01', 'Expected little-endian CDR String'
   n=struct.unpack_from('<I',data,4)[0]; assert data[8+n-1]==0
   obj=json.loads(data[8:8+n-1]);(udp if name.endswith('udp') else ev).append(obj)
native=[json.loads(x) for x in (r/'native-frames.jsonl').read_text().splitlines()]; native=[x for x in native if 0<x['sample_ns']<2**63]
stages={'native':native}
for s in ['sensor_receive','sensor_publish','detector_receive','buffer_take']:
 stages[s]=[x for x in ev if x['stage']==s]
lo=max(min(x['sample_ns'] for x in rows) for rows in stages.values());hi=min(max(x['sample_ns'] for x in rows) for rows in stages.values())
report={'common_source_window_s':[lo/1e9,hi/1e9],'stages':{}};sets={}
for s,rows in stages.items():
 d={x['sample_ns']:x for x in rows if lo<=x['sample_ns']<=hi};keys=sorted(d);gaps=[(b-a)/1e6 for a,b in zip(keys,keys[1:])];wall=(d[keys[-1]]['mono_ns']-d[keys[0]]['mono_ns'])/1e9
 report['stages'][s]={'frames':len(keys),'sim_hz':(len(keys)-1)/((keys[-1]-keys[0])/1e9),'wall_hz':(len(keys)-1)/wall,'median_sample_gap_ms':statistics.median(gaps),'max_sample_gap_ms':max(gaps),'max_wall_gap_ms':max((d[b]['mono_ns']-d[a]['mono_ns'])/1e6 for a,b in zip(keys,keys[1:]))};sets[s]=set(keys)
report['missing_at_next_stage']={a+' -> '+b:len(sets[a]-sets[b]) for a,b in zip(list(stages),list(stages)[1:])}
report['drop_reasons']=dict(collections.Counter(x.get('reason') for x in ev if x['stage']=='sensor_drop' and lo<=x['sample_ns']<=hi))
lat=[(x['forwarded_mono_ns']-x['mono_ns'])/1e6 for x in native];report['relay_ms']={'median':statistics.median(lat),'max':max(lat)}
sockets=collections.defaultdict(list)
for row in udp:
 for p in row['processes']:
  for s in p['sockets']:sockets[(row['role'],p['pid'],s['inode'])].append(s)
report['udp_sockets']=[dict(role=k[0],pid=k[1],inode=k[2],port=v[0]['port'],samples=len(v),first_drop=v[0]['drops'],last_drop=v[-1]['drops'],max_queue=max(x['rx_queue_bytes'] for x in v),rcvbuf=max(x.get('receive_buffer_bytes',0)for x in v))for k,v in sockets.items()]
(r/'native-rate-audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
