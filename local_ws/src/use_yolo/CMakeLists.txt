cmake_minimum_required(VERSION 3.0.2)
project(use_yolo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_filters
  tf
  actionlib
  control_msgs
  moveit_commander
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
    message_filters
    tf
    actionlib
    control_msgs
    moveit_commander
)

catkin_install_python(PROGRAMS
  scripts/xarm_vision_pick_drop.py
  scripts/yolo_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)