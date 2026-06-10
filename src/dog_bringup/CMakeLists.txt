cmake_minimum_required(VERSION 3.0.2)
project(dog_bringup)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/front_rear_cloud_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)