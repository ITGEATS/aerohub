"""Same-onboard-host cooperative motion gate; a crashed owner leaves a sticky record.

All motion Tools for a vehicle must use the same directory (shared mount across
containers). No expiry or automatic recovery: a process death is not a stop proof.
"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import threading


class MotionLease:
    def __init__(self, namespace, directory=None):
        namespace = '/' + namespace.strip('/')
        self.directory = Path(directory or os.environ.get('AEROHUB_MOTION_LOCK_DIR', '/tmp/aerohub-motion'))
        self.path = self.directory / (hashlib.sha256(namespace.encode()).hexdigest()+'.lock')
        self.namespace = namespace
        self.stream = None
        self.lock = threading.Lock()

    def owner(self):
        try:
            return self.path.read_text().strip()
        except FileNotFoundError:
            return ''

    def acquire(self, operation):
        with self.lock:
            if self.stream is not None:
                return False
            self.directory.mkdir(parents=True, exist_ok=True)
            stream = self.path.open('a+')
            try:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                stream.close()
                return False
            stream.seek(0)
            if stream.read().strip():
                stream.close()
                return False
            stream.write(json.dumps(dict(namespace=self.namespace, operation=operation, pid=os.getpid())))
            stream.flush()
            os.fsync(stream.fileno())
            self.stream = stream
            return True

    def release(self, stop_confirmed):
        with self.lock:
            if self.stream is None:
                return
            if stop_confirmed:
                self.stream.seek(0)
                self.stream.truncate()
                self.stream.flush()
                os.fsync(self.stream.fileno())
            self.stream.close()
            self.stream = None
