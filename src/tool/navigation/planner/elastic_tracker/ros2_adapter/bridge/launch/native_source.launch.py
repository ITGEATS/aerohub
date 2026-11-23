"""Explicit standalone entry. No planner framework process is launched."""
from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    path=LaunchConfiguration('native_config').perform(context)
    parameters=yaml.safe_load(Path(path).read_text())['planner']['parameters']
    inputs={key:parameters[key] for key in
            ('world_frame','odom_topic','target_topic','accepted_target_topic')}
    inputs['use_sim_time']=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    for key in ('proxy_host', 'state_source', 'depth_topic'):
        inputs[key] = LaunchConfiguration(key)
    inputs['proxy_port'] = ParameterValue(LaunchConfiguration('proxy_port'), value_type=int)
    return [
        Node(package='uav_elastic_tracker_adapter',executable='elastic_tracker_input_node',
             parameters=[inputs]),
        Node(package='uav_elastic_tracker_adapter',executable='elastic_tracker_native_node',
             name='elastic_native_adapter',parameters=[{
                 'resolved_config':LaunchConfiguration('native_config'),
                 'execution_mode':LaunchConfiguration('execution_mode'),
                 'execution_handoff':ParameterValue(LaunchConfiguration('execution_handoff'), value_type=bool),
                 'native_topic':LaunchConfiguration('native_topic'),
                 'proxy_host': LaunchConfiguration('proxy_host'),
                 'proxy_port': ParameterValue(LaunchConfiguration('proxy_port'), value_type=int),
                 'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}]),
    ]


def generate_launch_description():
    config=str(Path(get_package_share_directory('uav_elastic_tracker_adapter'))/'config/native_source.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('execution_mode',default_value='timed_path',choices=['timed_path']),
        DeclareLaunchArgument('execution_handoff',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('native_config',default_value=config),
        DeclareLaunchArgument('native_topic',default_value='',
                             description='Optional native ROS2 PolyTraj topic; empty retains RPC'),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('proxy_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('proxy_port', default_value=os.environ.get('ELASTIC_PROXY_PORT', '28500')),
        DeclareLaunchArgument('state_source', default_value='/drone0/sensor_measurements/odom'),
        DeclareLaunchArgument('depth_topic', default_value='/drone0/sensor_measurements/front_center/depth_planar/image'),
        OpaqueFunction(function=start),
    ])
