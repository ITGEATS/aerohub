cmake_minimum_required(VERSION 3.16)
project(uav_d2_slam)

find_package(ament_cmake REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_state_estimator REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(d2_slam_plugin SHARED src/plugin.cpp)
get_filename_component(AS2_ESTIMATOR_PREFIX "${as2_state_estimator_DIR}/../../.." ABSOLUTE)
find_path(AS2_ESTIMATOR_INCLUDE as2_state_estimator/plugin_base.hpp
  HINTS "${AS2_ESTIMATOR_PREFIX}/include" REQUIRED)
target_include_directories(d2_slam_plugin PRIVATE ${AS2_ESTIMATOR_INCLUDE})
file(STRINGS "${AS2_ESTIMATOR_INCLUDE}/as2_state_estimator/plugin_base.hpp"
  AS2_STATE_INTERFACE REGEX "getTransformationTypesAvailable")
if(AS2_STATE_INTERFACE)
  target_compile_definitions(d2_slam_plugin PRIVATE D2_SLAM_AS2_STATE_INTERFACE)
endif()
target_compile_features(d2_slam_plugin PUBLIC cxx_std_17)
ament_target_dependencies(d2_slam_plugin
  as2_core as2_state_estimator geometry_msgs nav_msgs pluginlib rclcpp tf2_geometry_msgs)
pluginlib_export_plugin_description_file(as2_state_estimator plugins.xml)

install(TARGETS d2_slam_plugin LIBRARY DESTINATION lib)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
