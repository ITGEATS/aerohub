#!/usr/bin/env python3
"""Workstation task: only System Tool operations; no Behavior/platform clients."""
import argparse
import json
from pathlib import Path
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from uav_system_python_api.client import SystemClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--drone-id', default='drone0')
    parser.add_argument('--height', type=float, default=3.)
    parser.add_argument('--use-wall-time', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('system_demo', namespace=args.drone_id,
                parameter_overrides=[Parameter('use_sim_time', value=not args.use_wall_time)])
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    tool = SystemClient(node)
    evidence = dict(success=False, calls=[], cleanup_errors=[])
    airborne = False

    def wait(future, timeout):
        deadline = time.monotonic()+timeout
        while not future.done():
            if time.monotonic()>deadline:
                raise TimeoutError('System Tool response deadline exceeded')
            time.sleep(.02)
        return future.result()

    def command(name, future, timeout):
        handle = wait(future, 10)
        if not handle.accepted:
            raise RuntimeError(name+' rejected')
        try:
            wrapped = wait(handle.get_result_async(), timeout+15)
        except BaseException:
            wait(handle.cancel_goal_async(), 10)
            raise
        result = wrapped.result
        entry = dict(operation=name, success=result.success, code=result.code,
                     reason=result.reason, stop_confirmed=result.stop_confirmed,
                     landed=result.landed, disarmed=result.disarmed, status=wrapped.status)
        evidence['calls'].append(entry)
        print(json.dumps(entry), flush=True)
        if not result.success or not result.stop_confirmed:
            raise RuntimeError(name+' failed: '+result.reason)
        return result

    try:
        deadline = time.monotonic()+60
        while not tool.ready:
            if time.monotonic()>deadline:
                raise TimeoutError('System Tool unavailable')
            time.sleep(.1)
        state = wait(tool.get_state(), 5)
        state_deadline = time.monotonic()+15
        while not state.fresh and time.monotonic()<state_deadline:
            time.sleep(.05)
            state = wait(tool.get_state(), 5)
        if not state.fresh or state.armed or state.busy:
            raise RuntimeError('fresh idle disarmed System Tool required')
        # Mark before submission; an uncertain outcome still requires cleanup.
        airborne = True
        command('takeoff', tool.takeoff(height=args.height), 60)
        command('land', tool.land(), 90)
        airborne = False
        state = wait(tool.get_state(), 5)
        evidence['success'] = state.fresh and not state.armed and not state.busy
    except BaseException as exc:
        evidence['error'] = str(exc)
    finally:
        if airborne:
            try:
                command('cleanup_land', tool.land(), 90)
            except BaseException as exc:
                evidence['cleanup_errors'].append(str(exc))
        (args.output/'result.json').write_text(json.dumps(evidence, indent=2))
        tool.destroy()
        executor.shutdown(timeout_sec=5)
        thread.join(5)
        node.destroy_node()
        rclpy.try_shutdown()
    if not evidence['success']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
