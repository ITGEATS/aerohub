"""Probe actual fresh depth delivery; independent of the flight-board supervisor."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
rclpy.init()
n = Node('flowpilot_sensor_health')
state = {}
n.create_subscription(Clock, '/clock', lambda m: state.update(clock=m.clock.sec+m.clock.nanosec/1e9), qos_profile_sensor_data)
n.create_subscription(Image, '/drone0/sensor_measurements/flowpilot_front/depth_planar/image', lambda m: state.update(image=m), qos_profile_sensor_data)
deadline = time.monotonic()+4
passed = False
while time.monotonic() < deadline:
    rclpy.spin_once(n, timeout_sec=.1)
    image = state.get('image')
    if image is not None and 'clock' in state:
        age = state['clock']-image.header.stamp.sec-image.header.stamp.nanosec/1e9
        passed = (image.encoding == '32FC1' and image.width == 320 and image.height == 185
                  and len(image.data) == image.step*image.height and 0 <= age <= .3)
        if passed: break
n.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if passed else 1)
