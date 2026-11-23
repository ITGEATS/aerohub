cmake_minimum_required(VERSION 3.8)
project(quadrotor_interfaces)
include(../source.cmake)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "${ELASTIC_SOURCE}/quadrotor_msgs:msg/AuxCommand.msg"
  "${ELASTIC_SOURCE}/quadrotor_msgs:msg/OccMap3d.msg"
  "${ELASTIC_SOURCE}/quadrotor_msgs:msg/PolyTraj.msg"
  "${ELASTIC_SOURCE}/quadrotor_msgs:msg/PositionCommand.msg"
  "${ELASTIC_SOURCE}/quadrotor_msgs:msg/Px4ctrlDebug.msg"
  "${ELASTIC_SOURCE}/quadrotor_msgs:msg/ReplanState.msg"
  "${ELASTIC_SOURCE}/quadrotor_msgs:msg/SO3Command.msg"
  "${ELASTIC_SOURCE}/quadrotor_msgs:msg/TakeoffLand.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
