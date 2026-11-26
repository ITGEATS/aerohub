"""Observe new platform messages before allowing owned-stack cleanup."""
import time
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.msg import PlatformInfo


def main():
    rclpy.init()
    node = Node('flowpilot_cleanup_gate')
    received = []
    node.create_subscription(PlatformInfo, '/drone0/platform/info',
                             lambda msg: received.append((time.monotonic(), msg.connected, msg.armed)),
                             qos_profile_sensor_data)
    deadline = time.monotonic() + 8
    def record(passed):
        path = Path('/evidence') / ('cleanup-platform-gate-' + str(time.time_ns()) + '.json')
        path.write_text(json.dumps(dict(passed=passed, wall=time.time(), samples=received), indent=2)+'\n')
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if len(received) >= 3 and all(connected and not armed for _, connected, armed in received[-3:]) and time.monotonic()-received[-1][0] < .5:
                print('Fresh platform samples confirm disarmed; owned stack may stop.')
                record(True)
                return 0
        print('Disarmed state unconfirmed: retain execution, PX4, clock and scene.')
        record(False)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
