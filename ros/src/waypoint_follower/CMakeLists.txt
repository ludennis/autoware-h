cmake_minimum_required(VERSION 2.8.3)
project(waypoint_follower)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    itri_msgs
    trace
    sensor_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        itri_msgs
        trace
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/controller.cpp
    src/global_path.cpp
    src/integrator.cpp
    src/move_forward_state.cpp
    src/move_reverse_state.cpp
    src/moving_range_matching.cpp
    src/parking.cpp
    src/signal_filter.cpp
    src/utils.cpp
    src/waypoint_follower.cpp
    src/waypoint_follower_node.cpp
    src/waypoint_follower_node_context.cpp
)

add_executable(${PROJECT_NAME}_node
    src/main.cpp
)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${catkin_LIBRARIES}
    PRIVATE
        ${JSONCPP_LIBRARIES}
        trace
)
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_node
    PRIVATE
        ${PROJECT_NAME}
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        -DDATA_DIR="${CMAKE_CURRENT_LIST_DIR}/data/")

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    add_library(${PROJECT_NAME}_test_file_loader
        STATIC
            test/file_loader/file_loader.cpp
    )
    target_include_directories(${PROJECT_NAME}_test_file_loader
        PUBLIC
            test/file_loader
        PRIVATE
            ${JSONCPP_INCLUDE_DIRS}
    )
    target_compile_definitions(${PROJECT_NAME}_test_file_loader
        PUBLIC
            -DTEST_DATA_DIR="${CMAKE_CURRENT_LIST_DIR}/test/data/"
    )
    target_link_libraries(${PROJECT_NAME}_test_file_loader
        PRIVATE
            ${JSONCPP_LIBRARIES}
    )

    catkin_add_gtest(${PROJECT_NAME}_test
        test/controller.cpp
        test/integrator.cpp
        test/main.cpp
        test/parking.cpp
        test/signal_filter.cpp
        test/state_machine.cpp
        test/waypoint_follower.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test
        PUBLIC
            ${catkin_LIBRARIES}
        PRIVATE
            ${PROJECT_NAME}
            ${PROJECT_NAME}_test_file_loader
    )
    target_include_directories(${PROJECT_NAME}_test
        PRIVATE
            src
    )
endif()
