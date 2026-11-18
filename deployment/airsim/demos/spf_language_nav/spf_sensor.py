"""Run the existing sensor adapter with a fresh-published-RGB health check."""
import json
from pathlib import Path
import sys
import time

STATE = Path('/tmp/spf-sensor-health.json')
MAX_AGE_S = 5.0


def healthy(path=STATE, now=None):
    try:
        state = json.loads(path.read_text())
        age = (time.monotonic() if now is None else now) - state['updated_monotonic']
        return state['stamp_ns'] > 0 and 0 <= age <= MAX_AGE_S
    except (OSError, ValueError, KeyError, TypeError):
        return False


def main():
    if sys.argv[1:] == ['healthcheck']:
        return 0 if healthy() else 1

    import os
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from uav_as2_sensor_adapter.node import SensorNode

    rclpy.init()
    node = None
    last_stamp = 0

    def heartbeat():
        nonlocal last_stamp
        # Both Image and CameraInfo have been published before the locked
        # adapter updates last_stamp. Duplicate samples do not refresh health.
        stamp = node.last_stamp.get(('FrontCamera', 0), 0)
        if stamp > last_stamp:
            temporary = STATE.with_suffix('.tmp')
            temporary.write_text(json.dumps(dict(
                pid=os.getpid(), stamp_ns=stamp, updated_monotonic=time.monotonic())))
            temporary.replace(STATE)
            last_stamp = stamp

    try:
        STATE.unlink(missing_ok=True)
        node = SensorNode()
        node.create_timer(.5, heartbeat)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        STATE.unlink(missing_ok=True)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
