"""Replay fixture for running the Planner without a Following Tool."""
import argparse
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from uav_elastic_tracker_adapter.core import split_ns


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['line', 'circle'], required=True)
    parser.add_argument('--origin', type=float, nargs=3, required=True, help='ENU; BEFORE upstream +1m offset')
    parser.add_argument('--velocity', type=float, nargs=3, help='line: world m/s')
    parser.add_argument('--radius', type=float, help='circle: metres')
    parser.add_argument('--angular-speed', type=float, help='circle: rad/s')
    parser.add_argument('--frame', default='drone0/odom')
    parser.add_argument('--topic', default='/elastic_tracker/target_odom')
    opts, ros_args = parser.parse_known_args(args)
    if opts.mode == 'line' and opts.velocity is None:
        parser.error('--velocity is required for line')
    if opts.mode == 'circle' and (opts.radius is None or opts.angular_speed is None):
        parser.error('--radius and --angular-speed are required for circle')
    numeric = opts.origin + (opts.velocity or []) + [v for v in (opts.radius, opts.angular_speed) if v is not None]
    if not all(map(math.isfinite, numeric)):
        parser.error('motion parameters must be finite')
    rclpy.init(args=ros_args)
    node = Node('elastic_tracker_target_stub')
    pub = node.create_publisher(Odometry, opts.topic, 10)
    epoch = None
    def tick():
        nonlocal epoch
        now = node.get_clock().now().nanoseconds
        if now == 0:  # ROS sim-clock uninitialized sentinel
            return
        if epoch is None:
            epoch = now
        t = (now - epoch) / 1_000_000_000
        if opts.mode == 'line':
            velocity = opts.velocity
            position = [a + t*b for a, b in zip(opts.origin, velocity)]
        else:
            angle = opts.angular_speed*t
            r, w = opts.radius, opts.angular_speed
            position = [opts.origin[0]+r*math.cos(angle), opts.origin[1]+r*math.sin(angle), opts.origin[2]]
            velocity = [-r*w*math.sin(angle), r*w*math.cos(angle), 0.0]
        msg = Odometry()
        msg.header.stamp.sec, msg.header.stamp.nanosec = split_ns(now)
        msg.header.frame_id, msg.child_frame_id = opts.frame, 'elastic_target'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = velocity
        pub.publish(msg)
    # 10 Hz is the user-specified minimum contract, not an invented acceptance rate.
    node.create_timer(1.0 / 10.0, tick)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
