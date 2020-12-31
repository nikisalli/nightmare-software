cmake_minimum_required(VERSION 3.0.2)
project(nightmare_usb_joystick)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  command.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  src/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})