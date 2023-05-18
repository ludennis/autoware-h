cmake_minimum_required(VERSION 2.8.3)
project(voxel_grid_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  velodyne_pointcloud
  itri_msgs
)

catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
  itri_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
  src/voxel_grid_filter_main.cpp
  src/voxel_grid_filter.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_compile_options(${PROJECT_NAME}_node
  PRIVATE
    -std=c++11
    -Wall
    -Werror
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
