#!/usr/bin/env python3
"""Send independent HUD RPC calls without importing the ProjectAirSim SDK."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('method')
    parser.add_argument('params', nargs='?', default='{}', help='JSON object')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8991)
    args = parser.parse_args()
    print(json.dumps(ProjectAirSimExtensionClient(args.host, args.port).call(args.method, **json.loads(args.params)),
                     ensure_ascii=False, indent=2))
