"""Forward original sampled state, target, depth and the sole ROS 2 clock."""
import base64
import copy
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from .transport import rpc
from .contracts import accept_odometry, InputRejected, InputDiagnostics, serialize_odometry as odometry
from .timing import trace


def header(value):
    return {'stamp': [value.stamp.sec, value.stamp.nanosec], 'frame_id': value.frame_id}


class Inputs(Node):
    def __init__(self):
        super().__init__('elastic_tracker_inputs')
        self.accepted_stamps = {}
        self.diagnostics = InputDiagnostics(self.get_logger())
        for key, default in (('proxy_host', '127.0.0.1'), ('proxy_port', int(os.environ.get('ELASTIC_PROXY_PORT', '28500'))),
                             ('world_frame', 'drone0/odom'),
                             ('odom_topic', '/drone0/elastic_tracker/odom'),
                             ('target_topic', '/elastic_tracker/target_odom'),
                             ('accepted_target_topic', '/elastic_tracker/accepted_target_odom'),
                             ('state_source', '/drone0/sensor_measurements/odom'),
                             ('depth_topic', '/drone0/sensor_measurements/front_center/depth_planar/image')):
            self.declare_parameter(key, default)
        self.create_subscription(Clock, '/clock', self.clock,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, self.get_parameter('target_topic').value,
                                 lambda m: self.send_odom('target', m), qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                 self.depth, qos_profile_sensor_data)
        self.world_odom = self.create_publisher(Odometry, self.get_parameter('odom_topic').value, 1)
        self.accepted_target = self.create_publisher(Odometry, self.get_parameter('accepted_target_topic').value, 1)
        self.local_pose = self.create_publisher(PoseStamped, '/drone0/elastic_tracker/local_pose', 1)
        self.create_subscription(Odometry, self.get_parameter('state_source').value,
                                 self.state, qos_profile_sensor_data)

    def state(self, original):
        try:
            accept_odometry(original, self.get_parameter('world_frame').value,
                            self.get_clock().now().nanoseconds, self.accepted_stamps.get('odom'))
        except InputRejected as exc:
            self.diagnostics.reject(str(exc))
            return
        msg = copy.deepcopy(original)
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm == 0:
            self.get_logger().error('state orientation cannot define a rotation')
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        v = original.twist.twist.linear
        rotation = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                    (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                    (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
        velocity = [sum(a*b for a, b in zip(row, (v.x, v.y, v.z))) for row in rotation]
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = velocity
        # ET-specific message uses world linear velocity, angular velocity remains body.
        # Covariance cannot keep its original basis after changing linear velocity.
        # Downstream ET ignores covariance; forward unchanged only on raw transport.
        msg.twist.covariance = [0.0] * 36  # not supplied for this mixed-expression contract
        if self.send_odom('odom', msg):
            self.world_odom.publish(msg)
            pose = PoseStamped()
            pose.header, pose.pose = original.header, original.pose.pose
            self.local_pose.publish(pose)

    def send(self, payload):
        try:
            rpc(self.get_parameter('proxy_host').value, self.get_parameter('proxy_port').value, payload)
            return True
        except (OSError, RuntimeError, ValueError) as exc:
            self.get_logger().error(str(exc))
            return False

    def clock(self, msg):
        self.send({'kind': 'clock', 'stamp': [msg.clock.sec, msg.clock.nanosec]})

    def send_odom(self, kind, msg):
        if kind == 'target':
            stamp = msg.header.stamp
            trace(self, 'target_input_received', stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec)
        try:
            accepted = accept_odometry(msg, self.get_parameter('world_frame').value,
                                      self.get_clock().now().nanoseconds, self.accepted_stamps.get(kind))
        except InputRejected as exc:
            self.diagnostics.reject(str(exc))
            return False
        # Both state and target contract explicitly carry WORLD linear velocity.
        forwarded = self.send({'kind': kind, 'message': accepted.payload})
        if forwarded:
            self.accepted_stamps[kind] = accepted.stamp_ns
        if forwarded and kind == 'target':
            trace(self, 'target_input_forwarded', stamp_ns=accepted.stamp_ns)
            self.accepted_target.publish(msg)
        return forwarded

    def depth(self, msg):
        if msg.encoding not in ('32FC1', '16UC1'):
            self.get_logger().error('ET mapping requires 32FC1 metres or 16UC1 millimetres')
            return
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        trace(self, 'depth_input_started', stamp_ns=stamp_ns, size_bytes=len(msg.data))
        data = base64.b64encode(bytes(msg.data)).decode()
        trace(self, 'depth_input_encoded', stamp_ns=stamp_ns)
        forwarded = self.send({'kind': 'depth', 'message': {'header': header(msg.header),
                  'height': msg.height, 'width': msg.width, 'encoding': msg.encoding,
                  'is_bigendian': msg.is_bigendian, 'step': msg.step,
                  'data': data}})
        trace(self, 'depth_input_finished', stamp_ns=stamp_ns, forwarded=forwarded)


def main(args=None):
    rclpy.init(args=args)
    node = Inputs()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
