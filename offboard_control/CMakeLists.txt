cmake_minimum_required(VERSION 3.0.2)
project(offboard_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
)
 
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs tf
)

catkin_install_python(PROGRAMS
  scripts/route_planner.py
  scripts/velocity_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

