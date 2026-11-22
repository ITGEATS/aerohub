"""Real ROS/AS2 type and node construction check; no action goals are sent.

Run in the existing Humble/AS2 environment after building this package and its
interface dependencies. This does NOT test flight, inference or convergence.
"""

import rclpy
from as2_msgs.action import GoToWaypoint
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, TwistStamped
from uav_aerovla_planner.goto_node import AerovlaGoToNode


def main():
    rclpy.init()
    node = None
    try:
        node = AerovlaGoToNode()
        assert not hasattr(node, 'trajectory_pub')
        assert not hasattr(node, 'ref_handoff')
        assert not node.has_parameter('roll_horizon_s')
        assert not node.has_parameter('valid_from_lead_s')
        assert node.phase == 'IDLE'
        assert node.state == 'IDLE'
        assert not node.pending_steps
        assert GoToWaypoint.Goal().yaw.FIXED_YAW == 2
        # No inference and no movement may begin just by constructing the node.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert node.in_flight is None and node.goal_handle is None
        assert node.get_parameter('front_topic').value == 'front'
        assert node.get_parameter('pose_topic').value == 'pose'
        node.pose = PoseStamped()
        node.twist = TwistStamped()
        node.pose.header.stamp = node.twist.header.stamp = node.get_clock().now().to_msg()
        node.pose.header.frame_id = 'earth'
        node.pose.pose.orientation.w = 0.0
        assert not node.state_fresh(), 'zero quaternion must be rejected'
        node.pose.pose.orientation.w = 1.0
        assert node.state_fresh()
        node.phase = 'READY'
        node.goal_frame = 'earth'
        node.pose.header.frame_id = 'other'
        assert not node.state_fresh(), 'frame change must be rejected'
        node.phase = 'IDLE'
        node.set_parameters([Parameter('enabled', value=False)])
        from uav_aerovla_planner.planner_node import AerovlaPlannerNode
        legacy = AerovlaPlannerNode()
        try:
            assert hasattr(legacy, 'trajectory_pub')
            assert legacy.has_parameter('roll_horizon_s')
            assert legacy.state == 'IDLE'
        finally:
            legacy.destroy_node()
        print('GOTO_ROS_CONSTRUCTION_PASS (GoTo has no trajectory resources; legacy construction preserved)')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
