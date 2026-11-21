#!/usr/bin/env python3
"""Pick the host address containers use to reach the Windows host.

Candidates are passed by run.py (host.docker.internal plus the Windows IPv4
adapters). A candidate wins only when BOTH the AirSim RPC TCP port (Blocks on
the Windows host) and the two inference-service health URLs answer from
inside the container network namespace. Prints the winner as JSON; run.py
records it in topology.json and rewrites planner.yaml with it.
"""
import json
import socket
import sys
import urllib.request

if len(sys.argv) < 5:
    raise SystemExit('usage: host_probe.py <rpc_port> <grounding_port> <clip_port> <candidate...>')
rpc_port, sam_port, clip_port = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
candidates = sys.argv[4:]

report = {}
for host in candidates:
    entry = {}
    try:
        with socket.create_connection((host, rpc_port), 4.0):
            entry['rpc_tcp'] = True
    except OSError as exc:
        entry['rpc_tcp'] = str(exc)
    for label, port in (('grounded_sam', sam_port), ('clip', clip_port)):
        try:
            with urllib.request.urlopen(
                    'http://%s:%d/health/ready' % (host, port), timeout=6) as resp:
                entry[label] = resp.status
        except Exception as exc:  # noqa: BLE001
            entry[label] = repr(exc)
    report[host] = entry
    if entry['rpc_tcp'] is True and entry['grounded_sam'] == 200 and entry['clip'] == 200:
        print(json.dumps({'chosen': host, 'report': report}))
        raise SystemExit(0)
print(json.dumps({'chosen': None, 'report': report}))
raise SystemExit(1)
