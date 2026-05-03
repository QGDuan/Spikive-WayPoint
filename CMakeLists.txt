cmake_minimum_required(VERSION 3.0.2)
project(waypoint_recorder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  controller_msgs
)

catkin_package()

install(PROGRAMS
  scripts/waypoint_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
