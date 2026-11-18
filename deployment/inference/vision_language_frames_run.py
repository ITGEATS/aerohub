#!/usr/bin/env python3
"""Compute-role supervisor: generic HTTP worker plus its public frames Function.

Same worker process as the single-image Function (serve.py exposes /infer and
/infer_frames); only the ROS adapter differs, so multi-eye Tools (CityNav) can
deploy the frames variant without duplicating the model environment.
"""
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


def main():
    model = ROOT / 'src/basic_function/inference/models/vision_language'
    python = os.environ.get('INFERENCE_WORKER_PYTHON', '/cloud-env/bin/python')
    namespace = os.environ.get('VEHICLE_NAMESPACE', 'drone0')
    parameters = os.environ['INFERENCE_ROS_PARAMETERS']
    commands = [
        [python, '-m', 'uvicorn', 'serve:app', '--host', '127.0.0.1', '--port', '8013'],
        ['python3', '-m', 'uav_inference.vision_language_frames_node', '--ros-args',
         '-r', '__ns:=/' + namespace, '--params-file', parameters],
    ]
    children = []
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        for command in commands:
            children.append(subprocess.Popen(command, cwd=model, start_new_session=True))
        while not stopping:
            if any(child.poll() is not None for child in children):
                raise RuntimeError('inference worker or ROS adapter exited')
            time.sleep(.1)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()


if __name__ == '__main__':
    main()
