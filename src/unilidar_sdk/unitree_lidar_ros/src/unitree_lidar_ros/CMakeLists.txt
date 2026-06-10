cmake_minimum_required(VERSION 3.0.2)
project(unitree_lidar_ros)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++14 -O3 -Wall)

get_filename_component(UNILIDAR_ROS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../unilidar_sdk/unitree_lidar_ros/src/unitree_lidar_ros" REALPATH)
get_filename_component(UNILIDAR_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../unilidar_sdk/unitree_lidar_sdk" REALPATH)
set(UNITREE_LIDAR_SDK_LIB "${UNILIDAR_SDK_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_lidar_sdk.a")

find_package(PCL REQUIRED QUIET)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  pcl_conversions
  roscpp
  roslib
  rosparam
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    pcl_conversions
    roscpp
    roslib
    rosparam
    rospy
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
  DEPENDS PCL
)

if(NOT EXISTS "${UNILIDAR_ROS_DIR}/src/unitree_lidar_ros_node.cpp")
  message(FATAL_ERROR "Unitree L1 ROS source not found: ${UNILIDAR_ROS_DIR}")
endif()

if(NOT EXISTS "${UNITREE_LIDAR_SDK_LIB}")
  message(FATAL_ERROR "Unitree L1 SDK library not found: ${UNITREE_LIDAR_SDK_LIB}")
endif()

include_directories(
  ${UNILIDAR_ROS_DIR}/include
  ${UNILIDAR_SDK_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(unitree_lidar_ros_node
  ${UNILIDAR_ROS_DIR}/src/unitree_lidar_ros_node.cpp
)
add_dependencies(unitree_lidar_ros_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(unitree_lidar_ros_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${UNITREE_LIDAR_SDK_LIB}
)

install(TARGETS unitree_lidar_ros_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)