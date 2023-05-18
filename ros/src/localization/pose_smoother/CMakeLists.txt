cmake_minimum_required(VERSION 2.8.3)
project(pose_smoother)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp geometry_msgs std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

target_compile_options(
    ${PROJECT_NAME}_node PUBLIC "-std=c++11;-Wall;-Werror"
)
