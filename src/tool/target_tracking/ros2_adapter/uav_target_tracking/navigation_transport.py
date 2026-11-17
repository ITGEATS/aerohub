"""Tracking's client of Navigation Tool; uses only its public Action/service."""
import math
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from uav_planner_interfaces.action import NavigateToPose
from uav_planner_interfaces.srv import UpdateNavigationTarget


class NavigationTransport:
    def __init__(self, node, group):
        self.node = node
        self.action = ActionClient(node, NavigateToPose, 'navigate_to_pose', callback_group=group)
        self.updates = node.create_client(UpdateNavigationTarget, 'navigate_to_pose/update_target', callback_group=group)

    def ready(self):
        return self.action.server_is_ready() and self.updates.service_is_ready()

    def target(self, point, frame, yaw):
        target = PoseStamped()
        target.header.frame_id = frame
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = point
        target.pose.orientation.z, target.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return target

    def start(self, point, frame, yaw, timeout, target_timeout):
        goal = NavigateToPose.Goal(target=self.target(point,frame,yaw),
            orientation_mode=NavigateToPose.Goal.TARGET_YAW,
            arrival_behavior=NavigateToPose.Goal.HOLD,
            use_timeout=True, timeout=float(timeout),
            use_target_timeout=True, target_timeout=float(target_timeout))
        return self.action.send_goal_async(goal)

    def update(self, handle, revision, point, frame, yaw):
        return self.updates.call_async(UpdateNavigationTarget.Request(goal_id=handle.goal_id,
            target_revision=revision, target=self.target(point,frame,yaw),
            orientation_mode=NavigateToPose.Goal.TARGET_YAW))
