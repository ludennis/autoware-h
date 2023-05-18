cmake_minimum_required(VERSION 2.8.3)
project(vehicle_gateway)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    itri_msgs
    trace
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

catkin_package(
    CATKIN_DEPENDS
        roscpp
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/vehicle_gateway_node.cpp
    src/eps_controller.cpp
    src/eps_diagnosis
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${catkin_LIBRARIES}
        ${JSONCPP_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        src
    PRIVATE
        ${JSONCPP_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        -DPARAM_FILE="${CMAKE_CURRENT_LIST_DIR}/param/param.json"
)

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    PRIVATE
        ${PROJECT_NAME}
)
