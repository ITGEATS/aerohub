#!/usr/bin/env python3
"""Check actor/flight clock coupling against a real, exclusively owned scene.

Loads/reloads the supplied native scene fixture. Never use against an
active flight: this test pauses the simulator and replaces its native scene.
Run with the matching ProjectAirSim Python environment.
"""
import argparse
import copy
import json
import math
import time
from pathlib import Path

from projectairsim import ProjectAirSimClient, World
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--topic-port', type=int, default=19889)
    parser.add_argument('--service-port', type=int, default=19890)
    parser.add_argument('--scene-port', type=int, default=19892)
    args = parser.parse_args()
    report = {'passed': False, 'checks': [], 'samples': []}
    rpc = ProjectAirSimExtensionClient(scene_port=args.scene_port)
    client = ProjectAirSimClient(port_topics=args.topic_port, port_services=args.service_port)
    client.connect()
    client.socket_services.recv_timeout = 10000
    world = None

    def sample():
        state = rpc.get_actor_state()
        state['native_sim_time_seconds'] = world.get_sim_time() * 1.e-9
        report['samples'].append({'wall': time.time(), **state})
        if 'clock_available' in state:
            assert state['clock_available'], 'Actor cannot read the native clock'
            assert abs(state['sim_time_seconds'] - state['native_sim_time_seconds']) < .1, 'Clock topic is stale'
        return state

    def check(name, condition, **metrics):
        report['checks'].append({'name': name, 'passed': bool(condition), **metrics})
        assert condition, f'{name}: {metrics}'

    def measure(label, seconds):
        start = sample()
        time.sleep(seconds)
        end = sample()
        sim = end['native_sim_time_seconds'] - start['native_sim_time_seconds']
        actor = end['time_seconds'] - start['time_seconds']
        check(label, sim > .2 and abs(actor - sim) < .08, sim_delta=sim,
              actor_delta=actor, actor_per_sim=actor / sim)

    try:
        world = World(client, args.scene.name, sim_config_path=str(args.scene.parent), delay_after_load_sec=2)
        baseline = copy.deepcopy(world.get_configuration())
        phase_position = None
        # A genuine failure-capable check: the old UE-time actor keeps moving
        # while this simulation clock is paused, and runs ~3x too fast below.
        for rate in (3000000, 9000000):
            if rate != 3000000:
                config = copy.deepcopy(baseline)
                config['clock']['real-time-update-rate'] = rate
                world.load_scene(config, delay_after_load_sec=2)
            world.pause()
            rpc.reset_actor()
            rpc.start_actor()
            # Equal native simulation intervals must produce the same sequence
            # phase AND actor position, despite different render/wall durations.
            world.continue_for_sim_time(3000000000, wait_until_complete=False)
            deadline = time.monotonic() + 20
            while not world.is_paused():
                assert time.monotonic() < deadline, 'Fixed simulation interval did not complete'
                time.sleep(.05)
            time.sleep(.3)
            phase = sample()
            check(f'fixed_sim_interval_phase_{rate}', abs(phase['time_seconds'] - 2.5) < .02,
                  expected_sequence_time=2.5, actual_sequence_time=phase['time_seconds'])
            position = [phase['location_ue_cm'][k] for k in ('x', 'y', 'z')]
            if phase_position is None:
                phase_position = position
            else:
                error_cm = math.dist(position, phase_position)
                check('equal_sim_time_equal_actual_position', error_cm < 1., position_difference_cm=error_cm)
            rpc.reset_actor()
            world.resume()
            rpc.start_actor()
            # Complete the one-second startup ramp in simulation time.
            start = sample()
            deadline = time.monotonic() + 15
            while sample()['native_sim_time_seconds'] - start['native_sim_time_seconds'] < 1.3:
                assert time.monotonic() < deadline, 'Simulation did not advance'
                time.sleep(.1)
            measure(f'coupled_rate_{rate}', 3)
            before = sample()
            repeated = rpc.start_actor()
            check(f'idempotent_start_{rate}', repeated['time_seconds'] >= before['time_seconds'] - .02
                  and not repeated['starting'])
            world.pause()
            time.sleep(.3)  # Let the final published clock/frame reach the sequence.
            frozen = sample()
            time.sleep(1)
            still = sample()
            drift = abs(still['time_seconds'] - frozen['time_seconds'])
            displacement = math.dist(list(still['location_ue_cm'].values()), list(frozen['location_ue_cm'].values()))
            check(f'sim_pause_freezes_actor_{rate}', drift < .001 and displacement < .1,
                  sequence_drift=drift, displacement_cm=displacement)
            world.resume()
            time.sleep(.3)
            measure(f'resumed_rate_{rate}', 2)
            stopped = rpc.stop_actor()
            time.sleep(.6)
            still = sample()
            check(f'actor_stop_{rate}', abs(still['time_seconds'] - stopped['time_seconds']) < .001)
            reset = rpc.reset_actor()
            check(f'actor_reset_{rate}', abs(reset['time_seconds']) < .001 and reset['state'] == 'paused')
        report['passed'] = True
    except Exception as exc:
        report['error'] = repr(exc)
        raise
    finally:
        try:
            if world:
                world.resume()
            rpc.stop_actor()
        finally:
            client.disconnect()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2))
    print('PASS: actor follows native simulation time at both rates, pause/resume/stop/reset/idempotence')


if __name__ == '__main__':
    main()
