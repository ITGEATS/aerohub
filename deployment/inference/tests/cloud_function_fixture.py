"""Synthetic compute-role fixture for cloud_cross_container.py, not a model baseline."""
import json
import os
from pathlib import Path
import signal
import threading

from test_cloud_function_ros import CloudFunctionContract

os.environ['INFERENCE_TEST_NAMESPACE'] = 'drone0'
fixture = CloudFunctionContract()
fixture.setUp()
fixture.release.set()
stop = threading.Event()
signal.signal(signal.SIGINT, lambda *_: stop.set())
signal.signal(signal.SIGTERM, lambda *_: stop.set())
try:
    Path('/cross/ready.json').write_text(json.dumps(dict(scope='synthetic',model_name='test-vlm'))+'\n')
    stop.wait(60)
finally:
    fixture.tearDown()
