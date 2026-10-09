cmake_minimum_required(VERSION 3.0.2)
project(ur10e_rg2_moveit)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  moveit_msgs
  std_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  HarvestPose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  moveit_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime moveit_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/harvest_pose_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)