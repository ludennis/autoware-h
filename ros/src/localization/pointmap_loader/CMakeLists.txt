cmake_minimum_required(VERSION 2.8.3)
project(pointmap_loader)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    sensor_msgs
    itri_msgs
)

find_package(PCL REQUIRED)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs itri_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(pointmap_loader
    src/pointmap_loader.cpp
    src/main.cpp
)
target_link_libraries(pointmap_loader
    ${catkin_LIBRARIES}
)
add_dependencies(pointmap_loader
    ${catkin_EXPORTED_TARGETS}
    itri_msgs_generate_messages_cpp
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pointmap_loader.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
