cmake_minimum_required(VERSION 3.8)
project(decomp_ros_interfaces)
include(../source.cmake)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "${ELASTIC_SOURCE}/planning/DecompROS/decomp_ros_msgs:msg/Ellipsoid.msg"
  "${ELASTIC_SOURCE}/planning/DecompROS/decomp_ros_msgs:msg/EllipsoidArray.msg"
  "${ELASTIC_SOURCE}/planning/DecompROS/decomp_ros_msgs:msg/Polyhedron.msg"
  "${ELASTIC_SOURCE}/planning/DecompROS/decomp_ros_msgs:msg/PolyhedronArray.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
