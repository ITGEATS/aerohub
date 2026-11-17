"""Compute-role supervisor for the SPF demo (AS2 + FollowTimedPath + spf planner).

VLM proxy runs in the workstation container (needs OpenCV); this supervisor
owns only the compute-role children. Task orchestration lives in mission.py.
"""
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[4]
APP = Path(__file__).resolve().parent


def main():
    commands = [
        ['python3', str(ROOT / 'deployment/airsim/as2/run.py')],
        ['ros2', 'run', 'uav_spf_planner', 'spf_planner', '--ros-args',
         '-r', '__ns:=/drone0', '--params-file', str(APP / 'planner.yaml')],
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
            children.append(subprocess.Popen(command, start_new_session=True))
        while not stopping:
            if children[0].poll() is not None:
                raise RuntimeError('AS2 execution process exited')
            time.sleep(.1)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=3)


if __name__ == '__main__':
    main()
