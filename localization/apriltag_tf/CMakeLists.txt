cmake_minimum_required(VERSION 3.0.2)
project(apriltag_tf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    apriltag_ros
)

find_package(Eigen3 REQUIRED)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs apriltag_ros
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ~/catkin_ws/src/orb_slam3_ros/orb_slam3/Thirdparty/Sophus
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node
  src/apriltag_tf_node.cpp
  src/apriltag_tf.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
