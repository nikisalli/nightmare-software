cmake_minimum_required(VERSION 3.0.2)
project(nightmare_state_broadcaster)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  src/broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})