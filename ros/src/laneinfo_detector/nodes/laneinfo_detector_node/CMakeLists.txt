add_executable(laneinfo_detector_node
    main.cpp
    node.cpp
    utils.cpp
    moving_range_matching.cpp
)

target_link_libraries(laneinfo_detector_node
    ${catkin_LIBRARIES}
)

add_dependencies(laneinfo_detector_node
    ${catkin_EXPORTED_TARGETS}
)
