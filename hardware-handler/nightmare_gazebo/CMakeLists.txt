cmake_minimum_required(VERSION 3.0.2)
project(nightmare_gazebo)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  src/joint_state_redirector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})