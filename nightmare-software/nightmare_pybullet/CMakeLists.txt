cmake_minimum_required(VERSION 3.0.2)
project(nightmare_pybullet)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  cv_bridge
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  src/nightmare_pybullet/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})