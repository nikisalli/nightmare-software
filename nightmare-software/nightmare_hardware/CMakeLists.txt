cmake_minimum_required(VERSION 3.0.0)
project(nightmare_hardware)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE cpp_files src/*.cpp)
message(STATUS ${cpp_files})
add_executable(${PROJECT_NAME} ${cpp_files})

# include common headers
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)