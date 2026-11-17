#!/usr/bin/env python3
"""Abandon the local course simulation, preserving evidence; no landing required."""
import json
import os
from pathlib import Path
import re
import signal
import subprocess

ROOT = Path(__file__).resolve().parents[4]
OWNED = re.compile(r'aerohub-(?:(?:yopo|ipc|diffphys)-course-(?:flight|compute)-[0-9-]+|manual5-(?:flight|compute|bag)-[0-9-]+|wmm5-flight)$')


def reset():
    import run_super as c
    root = ROOT/'log/projectairsim'
    # Read metadata before stopping any process; only accept evidence under our log root.
    records = []
    for name in ('yopo-course-session.json', 'diffphys-course-current.json', 'ipc-course-current.json'):
        source = root/name
        if not source.exists():
            continue
        record = json.loads(source.read_text())
        out = Path(record['output']).resolve()
        if out.parent != root.resolve():
            raise RuntimeError('Evidence ownership mismatch: '+str(out))
        records.append((record, out))
    active = c.capture(['docker','ps','--format','{{.Names}}']).splitlines()
    containers = [name for name in active if OWNED.fullmatch(name)]
    # Persist container console output before abandoning the instance.
    for name in containers:
        for record, out in records:
            if name in (record.get('flight'), record.get('compute')):
                result = subprocess.run(['docker','logs',name],capture_output=True,text=True)
                (out/(name+'-before-reset.log')).write_text(result.stdout+result.stderr)
    # Stop physics first so surviving clients cannot continue moving the old instance.
    if c.PX4 in active:
        c.run(['docker','stop','--timeout','3',c.PX4])
    if containers:
        c.run(['docker','stop','--timeout','2',*containers])
    c.manager('stop')
    for record, out in records:
        worker = record.get('worker')
        if worker is None and (out/'worker-process.json').exists():
            worker = json.loads((out/'worker-process.json').read_text())
        if worker:
            pid = int(worker['pid'])
            try:
                stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
                if stat[19] == worker['start_ticks'] and os.getpgid(pid) == pid:
                    os.killpg(pid,signal.SIGTERM)
            except (FileNotFoundError,ProcessLookupError):
                pass
        (out/'simulation-reset.json').write_text(json.dumps(dict(
            simulation_abandoned=True,landing_verified=False,evidence_preserved=True),indent=2)+'\n')
    print('本地赛道仿真已停止；无需降落或上锁。日志保留，重新运行任一 demo 即可。')


if __name__ == '__main__':
    reset()
