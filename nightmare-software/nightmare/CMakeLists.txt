cmake_minimum_required(VERSION 3.0.2)
project(nightmare)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf
  message_generation
)

catkin_python_setup()

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
  src/nightmare/hardware.py
  src/nightmare/engine.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})