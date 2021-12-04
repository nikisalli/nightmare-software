cmake_minimum_required(VERSION 3.0.2)
project(nightmare_engine)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  src/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})