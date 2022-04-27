cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11 -O3 -static-libstdc++)
set(CMAKE_BUILD_TYPE "Release")

# My lib.
add_library(mylibs_gnss_prep STATIC
  gnss_prep.cpp
  ${PROJECT_SOURCE_DIR}/include/qxh_com/qxh_com.cpp
  ${PROJECT_SOURCE_DIR}/include/config_io/config_io.cpp
  
  ${PROJECT_SOURCE_DIR}/include/GeographicLib/Math.cpp
  ${PROJECT_SOURCE_DIR}/include/GeographicLib/UTMUPS.cpp
  ${PROJECT_SOURCE_DIR}/include/GeographicLib/MGRS.cpp
  ${PROJECT_SOURCE_DIR}/include/GeographicLib/PolarStereographic.cpp
  ${PROJECT_SOURCE_DIR}/include/GeographicLib/TransverseMercator.cpp
  ${PROJECT_SOURCE_DIR}/include/GeographicLib/Utility.cpp
  )


add_dependencies(mylibs_gnss_prep ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

