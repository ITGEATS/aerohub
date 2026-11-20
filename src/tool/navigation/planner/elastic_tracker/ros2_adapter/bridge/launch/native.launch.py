"""Locked ROS 2 native algorithms and the compatible TCP endpoint; no motion."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def flatten(values, prefix=''):
    result = {}
    for key, value in values.items():
        name = prefix + key.replace('/', '.')
        if isinstance(value, dict):
            result.update(flatten(value, name + '.'))
        else:
            result[name] = value
    return result


def start(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    sim = get('use_sim_time').lower() == 'true'
    mapping = flatten(yaml.safe_load(Path(get('mapping_config')).read_text()))
    camera = flatten(yaml.safe_load(Path(get('camera_config')).read_text()))
    required = ('camera_rate', 'camera_range', 'cam_width', 'cam_height',
                'cam_fx', 'cam_fy', 'cam_cx', 'cam_cy', 'depth_scaling_factor',
                'cam2body_R', 'cam2body_p')
    missing = [key for key in required if key not in camera]
    if missing:
        raise ValueError('Missing camera calibration fields: ' + ', '.join(missing))
    for key, size in (('cam2body_R', 9), ('cam2body_p', 3)):
        if not isinstance(camera[key], list) or len(camera[key]) != size:
            raise ValueError('Invalid calibration vector: ' + key)
        camera[key] = [float(value) for value in camera[key]]
    mapping.update(camera)
    # ROS 1 accepted integral YAML values for double parameters. rclcpp's
    # declared type is strict; preserve values while expressing the native type.
    for key in ('local_x', 'local_y', 'local_z', 'camera_rate', 'camera_range',
                'cam_fx', 'cam_fy', 'cam_cx', 'cam_cy', 'depth_scaling_factor'):
        if key in mapping:
            mapping[key] = float(mapping[key])
    mapping['use_sim_time'] = sim
    if get('target_mask') != 'config':
        mapping['use_mask'] = get('target_mask') == 'true'
    planning = flatten(yaml.safe_load(Path(get('planner_config')).read_text()))
    planning['use_sim_time'] = sim
    if get('execution_handoff') == 'true':
        lead = float(planning.get('execution_handoff_lead_s', 0.2))
        if not 0.0 < lead < float('inf'):
            raise ValueError('execution_handoff_lead_s must be finite and positive')
        planning['execution_handoff_lead_s'] = lead
    else:
        planning['execution_handoff_lead_s'] = 0.0
    nodes = [
        Node(package='elastic_tracker_mapping', executable='mapping_node', name='mapping', namespace='elastic',
             parameters=[mapping], output='screen', remappings=[
                 ('/clock', '/elastic/native_clock'),
                 ('~/depth', '/elastic/depth'), ('~/odom', '/elastic/odom'),
                 ('~/target', '/elastic/target'), ('~/gridmap_inflate', '/elastic/gridmap_inflate')]),
        Node(package='elastic_tracker_planning', executable='planning_node', name='planning', namespace='elastic',
             parameters=[planning], output='screen', remappings=[
                 ('/clock', '/elastic/native_clock'),
                 ('~/odom', '/elastic/odom'), ('~/target', '/elastic/target'),
                 ('~/gridmap_inflate', '/elastic/gridmap_inflate'), ('~/trajectory', '/elastic/trajectory'),
                 ('~/heartbeat', '/elastic/heartbeat'), ('~/replanState', '/elastic/replanState'),
                 ('~/triger', '/triger')]),
        Node(package='uav_elastic_tracker_adapter', executable='elastic_tracker_native_rpc',
             parameters=[{'use_sim_time': sim, 'port': int(get('proxy_port')),
                          'fresh_native_output': get('fresh_native_output') == 'true'}], output='screen'),
    ]
    # Any native exit, including a clock rewind, tears down the complete session.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='Elastic native process exited; restart complete stack'))])) for node in nodes]
    return handlers + nodes


def generate_launch_description():
    config = Path(get_package_share_directory('uav_elastic_tracker_adapter')) / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('mapping_config', default_value=str(config / 'mapping.yaml')),
        DeclareLaunchArgument('planner_config', default_value=str(config / 'planner.yaml')),
        # No synthetic/default calibration can silently masquerade as the camera.
        DeclareLaunchArgument('camera_config', description='Absolute calibrated camera YAML'),
        DeclareLaunchArgument('target_mask', default_value='config', choices=['config', 'true', 'false']),
        DeclareLaunchArgument('proxy_port', default_value='28500'),
        DeclareLaunchArgument('fresh_native_output', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('execution_handoff', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start),
    ])
