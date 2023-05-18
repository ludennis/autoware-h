cmake_minimum_required(VERSION 2.8.3)
project(behavior)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  itri_msgs
  trace
  roslib
  btpp
  tf
)

find_package(PCL 1.2 REQUIRED)

option(PK_DEBUG "PK Debug" OFF)
option(OA_DEBUG "OA Debug" OFF)
option(LF_DEBUG "LF Debug" ON)
option(IA_DEBUG "IA Debug" OFF)
option(DEBUG_TREE "DEBUG_TREE Debug" OFF)
option(FSM_DEBUG "FSM Debug" OFF)

IF(PK_DEBUG)
    add_definitions("-DPK_DEBUG")
endif(PK_DEBUG)
IF(OA_DEBUG)
    add_definitions("-DOA_DEBUG")
endif(OA_DEBUG)
IF(LF_DEBUG)
    add_definitions("-DLF_DEBUG")
endif(LF_DEBUG)
IF(IA_DEBUG)
    add_definitions("-DIA_DEBUG")
endif(IA_DEBUG)
IF(DEBUG_TREE)
    add_definitions("-DDEBUG_TREE")
endif(DEBUG_TREE)
IF(FSM_DEBUG)
    add_definitions("-DFSM_DEBUG")
endif(FSM_DEBUG)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        btpp
        trace
        roscpp
        itri_msgs
        vehicle_gateway
        route_mission_handler
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(debug_to_file
  src/debug_to_file/debug_to_file.cpp
)

add_library(behavior
  src/state_machine_node.cpp
  src/core.cpp
  src/lane_following.cpp
  src/aeb.cpp
  src/state.cpp
  src/obstacle_avoidance.cpp
  src/behavior_parallel.cpp
  src/aeb_triggered_action.cpp
  src/get_curvature.cpp
  src/Avoidance/hsr_avoidance_sequence.cpp
  src/Avoidance/hsr_check_lane_condition.cpp
  src/Avoidance/hsr_obstacle_on_roadside_condition.cpp
  src/Avoidance/hsr_gen_bias_action.cpp
  src/Avoidance/hsr_reset_bias_action.cpp
  src/Avoidance/hsr_slow_down_action.cpp
  src/Avoidance/hsr_oa_trun_signal.cpp
  src/ACC/acc_action.cpp
  src/ACC/acc_condition.cpp
  src/ACC/acc_sequence.cpp
  src/Cruise/bumper_action.cpp
  src/Cruise/bumper_condition.cpp
  src/Cruise/cruise_fallback.cpp
  src/Cruise/curve_action.cpp
  src/Cruise/curve_condition.cpp
  src/Cruise/speed_limit_action.cpp
  src/TrafficLight/TL_Left_condition.cpp
  src/TrafficLight/TL_Left_sequence.cpp
  src/TrafficLight/TL_Red_condition.cpp
  src/TrafficLight/TL_Red_sequence.cpp
  src/TrafficLight/TL_Right_condition.cpp
  src/TrafficLight/TL_Right_sequence.cpp
  src/TrafficLight/traffic_light_fallback.cpp
  src/TrafficLight/traffic_light_action.cpp
  src/lane_change/find_current_lane_condition.cpp
  src/lane_change/find_current_lane_action.cpp
  src/lane_change/lane_change_action.cpp
  src/lane_change/lane_change_condition.cpp
  src/lane_change/lane_change_sequence.cpp
  src/lane_change/path_need_update_condition.cpp
  src/lane_change/update_path_action.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(behavior
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_dependencies(debug_to_file
  ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
    src/main.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
    behavior
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        debug_to_file
        ${PCL_LIBRARIES}
    PRIVATE
        ${JSONCPP_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PCL_INCLUDE_DIRS}
    PRIVATE
        ${JSONCPP_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        -DDATA_DIR="${CMAKE_CURRENT_LIST_DIR}/data/")

target_link_libraries(debug_to_file
    PRIVATE
        ${JSONCPP_LIBRARIES}
)
target_include_directories(debug_to_file
    PRIVATE
        ${JSONCPP_INCLUDE_DIRS}
)

target_compile_definitions(debug_to_file
    PRIVATE
        -DDEBUG_DATA_DIR="${CMAKE_CURRENT_LIST_DIR}/data/debug/")
