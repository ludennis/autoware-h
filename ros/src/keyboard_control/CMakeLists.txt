cmake_minimum_required(VERSION 2.8.3)
project(keyboard_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    itri_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        itri_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/keyboard_control_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
)
