cmake_minimum_required(VERSION 2.8.3)
project(robocontroller)


find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp)

add_library (robocontroller_lib rbctrlIface/rbctrlIface.cpp)

add_executable(robocontroller robocontroller_node.cpp)
target_link_libraries( robocontroller ${catkin_LIBRARIES})

