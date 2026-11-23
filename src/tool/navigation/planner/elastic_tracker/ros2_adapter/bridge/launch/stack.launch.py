"""Native + existing inputs/execution; AS2 and task lifecycle remain external."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start(context):
    launch = Path(get_package_share_directory('uav_elastic_tracker_adapter')) / 'launch'
    native_topic = LaunchConfiguration('native_topic').perform(context)
    execution_handoff = 'true' if native_topic else 'false'
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch / 'native.launch.py')),
                                 launch_arguments={'camera_config': LaunchConfiguration('camera_config'),
                                                   'use_sim_time': LaunchConfiguration('use_sim_time'),
                                                   'target_mask': LaunchConfiguration('target_mask'),
                                                   'fresh_native_output': LaunchConfiguration('fresh_native_output'),
                                                   'execution_handoff': execution_handoff}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch / 'native_source.launch.py')),
                                 launch_arguments={'native_topic': native_topic,
                                                   'native_config': LaunchConfiguration('native_config'),
                                                   'use_sim_time': LaunchConfiguration('use_sim_time'),
                                                   'depth_topic': LaunchConfiguration('depth_topic'),
                                                   'execution_handoff': execution_handoff}.items()),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('uav_elastic_tracker_adapter')) / 'config/native_source.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('camera_config', description='Absolute calibrated camera YAML'),
        DeclareLaunchArgument('native_config', default_value=str(config)),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('depth_topic', default_value='/drone0/sensor_measurements/front_center/depth_planar/image'),
        DeclareLaunchArgument('target_mask', default_value='config', choices=['config', 'true', 'false']),
        DeclareLaunchArgument('fresh_native_output', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('native_topic', default_value='/elastic/trajectory',
                             description='ROS2 direct output; empty explicitly selects compatibility RPC'),
        OpaqueFunction(function=start),
    ])
