cmake_minimum_required(VERSION 3.0.2)
project(nightmare_pybullet)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    cv_bridge
)

catkin_python_setup()