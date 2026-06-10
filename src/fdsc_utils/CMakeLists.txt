cmake_minimum_required(VERSION 3.0.2)
project(fdsc_utils)

add_compile_options(-std=c++17 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fdsc_utils_sdk
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(FDSC_SDK_SOURCES
  src/common.cpp
  src/unitreeConnectBoost.cpp
  src/lowState.cpp
  src/lowCmd.cpp
  src/highCmd.cpp
  src/highState.cpp
)

add_library(fdsc_utils_sdk ${FDSC_SDK_SOURCES})
target_link_libraries(fdsc_utils_sdk
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(unitree_highlevel_bridge
  src/unitree_highlevel_bridge.cpp
)
target_link_libraries(unitree_highlevel_bridge
  fdsc_utils_sdk
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS fdsc_utils_sdk unitree_highlevel_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)