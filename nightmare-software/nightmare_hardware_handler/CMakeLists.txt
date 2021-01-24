cmake_minimum_required(VERSION 3.0.2)
project(nightmare_hardware_handler)

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
  src/hardware-handler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})