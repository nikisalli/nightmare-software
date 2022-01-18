cmake_minimum_required(VERSION 3.0.2)
project(nightmare)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  src/nightmare/hardware.py
  src/nightmare/engine.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})