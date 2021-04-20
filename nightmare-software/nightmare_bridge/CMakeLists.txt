cmake_minimum_required(VERSION 3.0.2)
project(nightmare_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_msgs
)

find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                 octomap_msgs
  DEPENDS OCTOMAP
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nightmare_bridge src/main.cpp)
add_dependencies(nightmare_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)