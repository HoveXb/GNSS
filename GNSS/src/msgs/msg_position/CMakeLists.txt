cmake_minimum_required(VERSION 2.8.3)
project(msg_position)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3 -static)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  msg_position.msg
  msg_position_long.msg)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs)


catkin_package(CATKIN_DEPENDS message_runtime)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

