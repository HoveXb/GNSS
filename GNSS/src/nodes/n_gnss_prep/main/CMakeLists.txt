cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11 -O3 -static-libstdc++)
set(CMAKE_BUILD_TYPE "Release")

# As main func.
add_executable(n_gnss_prep n_gnss_prep.cpp)
target_link_libraries(n_gnss_prep ${catkin_LIBRARIES})
target_link_libraries(n_gnss_prep mylibs_gnss_prep)

#add_dependencies(n_radar_obj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
