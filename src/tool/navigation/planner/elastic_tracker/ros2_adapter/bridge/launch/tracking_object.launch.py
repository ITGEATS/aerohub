"""Run the shared RGB-D detector and TrackObject with Elastic as motion owner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    query = ParameterValue(LaunchConfiguration('query'), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('query', default_value='person'),
        DeclareLaunchArgument('detector_config', description='ROS parameters for the registered RGB-D detector'),
        DeclareLaunchArgument('max_capture_age', default_value='0.5',
                              description='Maximum age of a detection when the tracker consumes it, in seconds'),
        DeclareLaunchArgument('max_prediction_time', default_value='0.5',
                              description='Maximum age of the last associated observation in a valid target state'),
        Node(package='uav_object_detection', executable='object_detection', namespace=namespace,
             parameters=[LaunchConfiguration('detector_config'), {
                 'use_sim_time': sim_time,
                 'stream_enabled': True,
                 'query': query,
                 'target_class': query,
             }], remappings=[
                 ('inference/yolo/infer', '/inference/yolo/infer'),
                 ('inference/yolo/status', '/inference/yolo/status'),
             ], output='screen'),
        Node(package='uav_target_tracking', executable='target_tracking', namespace=namespace,
             parameters=[{
                 'use_sim_time': sim_time,
                 'observation_mode': 'stream',
                 'stream_query': query,
                 'max_capture_age': ParameterValue(
                     LaunchConfiguration('max_capture_age'), value_type=float),
                 'max_prediction_time': ParameterValue(
                     LaunchConfiguration('max_prediction_time'), value_type=float),
             }], output='screen'),
        Node(package='uav_elastic_tracker_adapter', executable='elastic_tracking_bridge',
             namespace=namespace, parameters=[{'use_sim_time': sim_time}], output='screen'),
    ])
