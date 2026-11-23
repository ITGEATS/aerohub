"""Connect Elastic Following to already deployed Tracking and Planner services."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        Node(package='uav_elastic_tracker_adapter', executable='elastic_following',
             namespace=LaunchConfiguration('namespace'),
             parameters=[{'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}],
             output='screen'),
    ])
