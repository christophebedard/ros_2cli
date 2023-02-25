cmake_minimum_required(VERSION 3.8)
project(ros_2cli)

find_package(ament_cmake REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ros-2cli_path.sh")

install(
  PROGRAMS bin/ROS
  DESTINATION share/${PROJECT_NAME}/bin)
install(
  PROGRAMS bin/ros
  DESTINATION share/${PROJECT_NAME}/bin)

ament_package()
