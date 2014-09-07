cmake_minimum_required(VERSION 2.8.3)

project(robocontroller)

set(ROS_BUILD_TYPE Debug)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(robocontroller_lib
            rbctrliface/rbctrliface.cpp
            rbctrliface/robotctrl.cpp )

add_executable(robocontroller_node robocontroller_node.cpp)

target_link_libraries(robocontroller_node ${catkin_LIBRARIES} robocontroller_lib modbus)

