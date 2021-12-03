cmake_minimum_required(VERSION 3.0.0)
project(nightmare_engine)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE cpp_files src/*.cpp)
add_executable(${PROJECT_NAME} ${cpp_files})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)