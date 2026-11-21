"""Adapt the public Tracking Tool state to Elastic's target Odometry."""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from uav_target_tracking_interfaces.msg import TargetState

from .contracts import InputRejected, accept_odometry
from .timing import trace


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class TargetStateBridge(Node):
    def __init__(self):
        super().__init__('elastic_tracking_bridge')
        namespace = self.get_namespace().strip('/')
        frame = f'{namespace}/odom' if namespace else 'odom'
        self.frame = self.declare_parameter('world_frame', frame).value
        source = self.declare_parameter('target_state_topic', 'tools/target_tracking/state').value
        target = self.declare_parameter('target_topic', '/elastic_tracker/target_odom').value
        self.output = self.create_publisher(Odometry, target, 1)
        self.create_subscription(TargetState, source, self.forward, QoSProfile(depth=1))
        self.session = None
        self.generation = 0
        self.last_stamp_ns = 0
        self.stopped = False

    def forward(self, state):
        stamp = stamp_ns(state.header.stamp)
        expires = stamp_ns(state.valid_until)
        if hasattr(state, 'last_observation_stamp'):
            session = bytes(state.session_id.uuid)
            if not any(session):
                return
            observed = stamp_ns(state.last_observation_stamp)
            usable = state.velocity_valid and state.state in (state.TRACKING, state.COASTING)
            stopped = state.state == state.STOPPED
        else:
            session = state.session_id
            observed = stamp_ns(state.observation_stamp)
            usable = state.status in ('tracked', 'predicted')
            stopped = state.status == 'stopped'
        if state.header.frame_id != self.frame or not session or state.generation == 0:
            return
        if session != self.session:
            self.session = session
            self.generation = 0
            self.stopped = False
        if self.stopped or state.generation < self.generation:
            return
        if state.generation > self.generation:
            self.generation = state.generation
            self.last_stamp_ns = 0
        if stopped:
            self.stopped = True
        if (not state.valid or not usable or not 0 < observed <= stamp <= expires
                or self.get_clock().now().nanoseconds > expires):
            return

        message = Odometry(header=state.header, child_frame_id=self.frame)
        message.pose.pose.position = state.position
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear = state.velocity
        for row in range(3):
            for column in range(3):
                index = 6 * row + column
                message.pose.covariance[index] = state.covariance[index]
                velocity_index = 6 * (row + 3) + column + 3
                message.twist.covariance[index] = state.covariance[velocity_index]
        try:
            accepted = accept_odometry(message, self.frame)
        except InputRejected:
            return
        if accepted.stamp_ns > self.last_stamp_ns:
            self.last_stamp_ns = accepted.stamp_ns
            trace(self, 'target_bridge_publish', stamp_ns=accepted.stamp_ns,
                  observation_ns=observed, valid_until_ns=expires)
            self.output.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TargetStateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
