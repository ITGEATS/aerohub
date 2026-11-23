"""Measurement only: append provenance events without feeding control decisions."""
import json
import os
from pathlib import Path
import time


def record(node, source, kind, **fields):
    value = dict(kind=kind, source=source, ros_ns=node.get_clock().now().nanoseconds,
                 monotonic_ns=time.monotonic_ns(), **fields)
    directory = os.environ.get('ELASTIC_EVIDENCE_DIR')
    if directory:
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        with (path/(source+'.jsonl')).open('a') as stream:
            stream.write(json.dumps(value, allow_nan=False)+'\n')
    return value
