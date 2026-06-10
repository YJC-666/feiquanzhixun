cmake_minimum_required(VERSION 3.0.2)
project(web_pointcloud_ground_station)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/static_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)