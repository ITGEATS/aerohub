"""Name the exact native Action stage when a task wait expires."""
import time


def wait_action_stage(mission, predicate, name, stage, timeout):
    started = time.monotonic()
    clock_before = mission.last_clock_ns
    try:
        mission.wait(predicate, timeout)
    except TimeoutError as error:
        platform = mission.platform
        received_at = getattr(mission, 'clock_received_at', None)
        clock_age = None if received_at is None else round(time.monotonic()-received_at, 2)
        raise TimeoutError(
            f'{name}: timeout waiting for {stage} after {time.monotonic()-started:.1f}s; '
            f'sim_clock_advanced={mission.last_clock_ns != clock_before}; '
            f'clock_age_s={clock_age}; '
            f'armed={getattr(platform, "armed", None)}; '
            f'offboard={getattr(platform, "offboard", None)}; '
            'keep simulator window open until landing and disarm complete'
        ) from error
