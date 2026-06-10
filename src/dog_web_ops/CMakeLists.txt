cmake_minimum_required(VERSION 3.0.2)
project(dog_web_ops)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/static_server.py
    scripts/camera_preview_node.py
    scripts/qr_inspection_detector_node.py
    scripts/qr_jichu_trigger_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)