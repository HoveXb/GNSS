cmake_minimum_required(VERSION 2.8.3)
project(n_gnss_prep)

# Compile as C++11, supported in ROS Kinetic and newer
# And optimize the code with level O3.
add_compile_options(-std=c++14 -O3 -static-libstdc++)
set(CMAKE_BUILD_TYPE "Release")


catkin_package(CATKIN_DEPENDS message_runtime)

# Find packages and libs
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs message_generation
  msg_position  # For needed msg. Only this line is necessary if one msg is needed for the node!
  )

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

# Add sub-dirs
# Note: new CMakeLists.txt files should exist in the sub-dirs.
add_subdirectory(src)
add_subdirectory(main)




