cmake_minimum_required(VERSION 2.8.3)
project(nmea2pose)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nmea_msgs
    tf
    pcl_ros
    trace
)

find_package(PCL REQUIRED)
find_package(GeographicLib REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs nmea_msgs tf trace
    LIBRARIES coord_conv
    DEPENDS GeographicLib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(coord_conv
    src/geo_pos_conv.cpp
)
target_link_libraries(coord_conv ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

add_executable(nmea2pose
    src/nmea2pose.cpp
    src/main.cpp
)
target_link_libraries(nmea2pose ${catkin_LIBRARIES} coord_conv)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_coord_conv_test
        test/geo_pos_conv.cpp
        src/geo_pos_conv.cpp)
    target_link_libraries(${PROJECT_NAME}_coord_conv_test ${GeographicLib_LIBRARIES})
    target_include_directories(${PROJECT_NAME}_coord_conv_test PRIVATE include)
endif()

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_test
        test/nmea2pose.cpp
        src/nmea2pose.cpp)
    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} coord_conv)
    target_include_directories(${PROJECT_NAME}_test PRIVATE include)
endif()
