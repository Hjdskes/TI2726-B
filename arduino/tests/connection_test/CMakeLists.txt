cmake_minimum_required(VERSION 2.8.3)
project(es)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(blink blink.cpp)
target_link_libraries(blink ${catkin_LIBRARIES})
add_dependencies(blink es_generate_messages_cpp)
