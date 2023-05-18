cmake_minimum_required(VERSION 2.8.3)
project(fake_traffic_light)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    itri_msgs
    roscpp
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        itri_msgs
        roscpp
        std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/node.cpp
)

add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
