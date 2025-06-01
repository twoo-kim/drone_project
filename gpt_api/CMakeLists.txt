cmake_minimum_required(VERSION 3.0.2)
project(gpt_api)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GPTAsk.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/goal_selector.py
  scripts/gpt_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
