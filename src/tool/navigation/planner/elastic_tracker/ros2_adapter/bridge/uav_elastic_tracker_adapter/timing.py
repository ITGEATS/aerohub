"""Opt-in asynchronous timing trace; never changes RPC or trajectory payloads."""
import atexit
import json
import os
from pathlib import Path
import queue
import threading
import time

_directory = os.environ.get('ELASTIC_TIMING_TRACE')
_events = queue.SimpleQueue()
_writer = None
_lock = threading.Lock()


def _write(stream):
    with stream:
        while True:
            event = _events.get()
            if event is None:
                break
            stream.write(json.dumps(event, separators=(',', ':'))+'\n')
            stream.flush()


def trace(node, event, **fields):
    if not _directory:
        return
    wall = time.monotonic_ns()
    global _writer
    if _writer is None:
        with _lock:
            if _writer is None:
                path = Path(_directory)
                path.mkdir(parents=True, exist_ok=True)
                stream = (path/(str(os.getpid())+'.jsonl')).open('a')
                _writer = threading.Thread(target=_write, args=(stream,), daemon=True)
                _writer.start()
    _events.put(dict(event=event, mono_ns=wall, wall_ns=time.time_ns(), pid=os.getpid(),
                     node=node.get_name() if node else None,
                     ros_ns=node.get_clock().now().nanoseconds if node else None,
                     **fields))


@atexit.register
def close():
    if _writer is not None:
        _events.put(None)
        _writer.join()
