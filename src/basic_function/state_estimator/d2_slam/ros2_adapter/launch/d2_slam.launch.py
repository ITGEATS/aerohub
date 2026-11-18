from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    self_id = LaunchConfiguration("self_id")
    main_id = LaunchConfiguration("main_id")
    config = LaunchConfiguration("config")
    lcm_uri = LaunchConfiguration("lcm_uri")

    native_launch = PathJoinSubstitution(
        [FindPackageShare("d2vins"), "launch", "d2_slam.launch.py"])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="drone0"),
        DeclareLaunchArgument("self_id", default_value="0"),
        DeclareLaunchArgument("main_id", default_value="0"),
        DeclareLaunchArgument(
            "config", default_value="/opt/d2_slam/config/d2_slam.yaml"),
        DeclareLaunchArgument(
            "lcm_uri", default_value="udpm://224.0.0.251:7667?ttl=1"),
        Node(
            package="uav_d2_slam_ros2",
            executable="d2_slam_adapter",
            name="d2_slam_adapter",
            namespace=namespace,
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(native_launch),
            launch_arguments={
                "namespace": namespace,
                "self_id": self_id,
                "main_id": main_id,
                "config": config,
                "lcm_uri": lcm_uri,
                "enable_loop": "true",
                "enable_pgo": "true",
                "enable_comm": "true",
            }.items(),
        ),
    ])
