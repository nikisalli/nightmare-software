cmake_minimum_required(VERSION 2.8.3)
project(nightmare)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  robot_state_publisher
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_install_python(PROGRAMS 
  src/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})