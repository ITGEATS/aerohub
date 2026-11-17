"""Assign a unique mapped UDP participant to each PC compute child, then exec."""
import fcntl
import json
import os
from pathlib import Path
import sys
sys.path.insert(0,'/single')
from dds_mapped import mapped_profile

root=Path(os.environ['EGO_SESSION_ROOT'])
config=json.loads((root/'config.json').read_text())
out=Path(os.environ['EGO_GROUND_STATE'])/'dds'
out.mkdir(exist_ok=True)
with (out/'allocation.lock').open('a') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX)
    index=out/'assignments.json'
    rows=json.loads(index.read_text()) if index.exists() else []
    participant=7+len(rows)
    if participant>(17 if config.get('fault_case') else 16):raise RuntimeError('PC compute participant pool exhausted')
    path=out/(str(participant)+'.xml')
    mapped_profile(path,config['pc_ip'],config['board_ip'],config['domain'],participant)
    rows.append(dict(participant=participant,pid=os.getpid(),argv=sys.argv[1:]))
    index.write_text(json.dumps(rows,indent=2))
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(path)
os.execvp(sys.argv[1],sys.argv[1:])
