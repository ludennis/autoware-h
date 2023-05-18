cmake_minimum_required(VERSION 2.8.3)
project(motion_planner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    itri_msgs
    roscpp
    roslib
    tf
    trace
    visualization_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        itri_msgs
        roscpp
        roslib
        tf
        trace
        visualization_msgs
)

###########
## Build ##
###########
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC
    src/along_global_path_state.cpp
    src/cost_function.cpp
    #src/custom_path_state.cpp
    src/global_path.cpp
    src/idle_state.cpp
    src/motion_planner_node.cpp
    src/moving_range_matching.cpp
)

add_library(${PROJECT_NAME}_utils STATIC
    src/utils.cpp
)

add_dependencies(${PROJECT_NAME}_utils
    ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    PRIVATE
        ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_utils
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
    add_library(${PROJECT_NAME}_test_file_loader
        STATIC
            test/file_loader/file_loader.cpp
    )

    add_dependencies(${PROJECT_NAME}_test_file_loader
      ${catkin_EXPORTED_TARGETS}
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
        test/cost_function.cpp
        test/global_path.cpp
        test/main.cpp
        test/moving_range_matching.cpp
        test/state_machine.cpp
        test/utils.cpp
    )

    target_link_libraries(${PROJECT_NAME}_test
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        ${PROJECT_NAME}_test_file_loader
    )

    target_compile_definitions(${PROJECT_NAME}_test
        PRIVATE
            -DTEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test/")

    target_include_directories(${PROJECT_NAME}_test PRIVATE src)
endif()
