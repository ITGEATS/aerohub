"""Session-only, single-owner planner-input fault gate; AS2 inputs untouched."""
import importlib.util
import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
ROOT=Path(os.environ.get('EGO_SESSION_ROOT',Path(__file__).resolve().parent))
def nodes(context):
    config=json.loads((ROOT/'config.json').read_text())
    vehicle=LaunchConfiguration('vehicle').perform(context)
    assert vehicle==config['fault_vehicle']==config['vehicle']
    path=Path(config['ego_launch_file']) if config.get('ego_launch_file') else Path(get_package_share_directory('uav_ego_planner_adapter'))/'launch/ego_as2.launch.py'
    spec=importlib.util.spec_from_file_location('original_ego_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=module.nodes(context)
    result[-1]=Node(package='uav_ego_planner_adapter',executable='ego_as2_adapter',namespace=vehicle,
        parameters=[{'resolved_config':LaunchConfiguration('resolved_config').perform(context),'vehicle':vehicle,
            'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],
        remappings=[('/'+vehicle+'/navigation/'+suffix,'/acceptance/'+vehicle+'/'+suffix) for suffix in ('odom','points')],output='screen')
    # ExecuteProcess inherits ros2 launch --launch-prefix just like Node.
    # Wrapping it here again allocates two participants for the same PID.
    cmd=['python3',str(ROOT/'fault_injector.py')]
    result.append(ExecuteProcess(cmd=cmd,output='screen'))
    return result
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('resolved_config'),DeclareLaunchArgument('vehicle'),
        DeclareLaunchArgument('use_sim_time',default_value='true'),OpaqueFunction(function=nodes)])
