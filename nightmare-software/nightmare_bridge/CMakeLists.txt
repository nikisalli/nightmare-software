cmake_minimum_required(VERSION 3.0.2)
project(nightmare_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)