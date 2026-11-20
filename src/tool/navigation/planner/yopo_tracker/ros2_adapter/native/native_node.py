"""Launch the pinned upstream node under the Humble interpreter."""
from pathlib import Path
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.type_support import check_for_type_support
from rclpy.utilities import remove_ros_args
from quadrotor_msgs.msg import PositionCommand


def main():
    upstream = Path(__file__).resolve().parents[2] / 'upstream/YOPO'
    sys.path.insert(0, str(upstream))
    from test_yopo_ros import YopoNet

    check_for_type_support(PositionCommand)
    # Match the upstream demo's algorithm settings for the migration baseline.
    settings = dict(
        use_tensorrt=False, pitch_angle_deg=0, image_fps=30, simulation=True,
        conf_thresh=0.9, ekf_thresh=2.0, plan_from_reference=False,
        verbose=False, visualize=True,
        odom_topic='~/odom', depth_topic='~/depth', rgb_topic='~/rgb',
        ctrl_topic='~/diagnostics/position_command',
    )
    ros_args = sys.argv[1:]
    if remove_ros_args(args=['native_node', *ros_args]) != ['native_node']:
        raise SystemExit('Pass ROS remapping and parameter arguments after --ros-args')
    rclpy.init(args=[
        '--ros-args',
        '-r', '/respawn:=~/respawn',
        *[rule for topic in ('lattice_trajs_visual', 'best_traj_visual', 'trajs_visual',
                             'target_pred', 'target_ekf', 'target_image')
          for rule in ('-r', f'/yopo_net/{topic}:=~/diagnostics/{topic}')],
        '--',
        *ros_args,
    ])
    try:
        node = YopoNet(settings, str(upstream / 'saved/YOPO_1/epoch50.pth'))
        try:
            node.get_logger().info(f'YOPO-Tracker initialized on {node.device}')
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
