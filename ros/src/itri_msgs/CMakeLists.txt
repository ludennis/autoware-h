cmake_minimum_required(VERSION 2.8.3)
project(itri_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    sensor_msgs
    std_msgs
)

add_message_files(
    FILES
        Ars40xObject.msg
        Ars40xObjects.msg
        BehaviorState.msg
        blinker_cmd.msg
        CarState.msg
        ConfigVoxelGridFilter.msg
        ConvexHull.msg
        DetectedObject.msg
        DetectedObjectArray.msg
        GPS.msg
        Lane.msg
        lane_info.msg
        lane_waypoints_info.msg
        NdtStatistics.msg
        Nroad.msg
        Path.msg
        PathSegment.msg
        plan.msg
        PointsMapInfo.msg
        VoxelGridFilterInfo.msg
        PointXYZSD.msg
        Polygon.msg
        Route_info.msg
        speed_cmd.msg
        StationInfo.msg
        steer_cmd.msg
        TrafficLightObject.msg
        TrafficLightObjects.msg
        turn_signal_cmd.msg
        VehicleState.msg
        Waypoint.msg
        WaypointArray.msg
        WaypointArrays.msg
)

generate_messages(
    DEPENDENCIES
        geometry_msgs
        sensor_msgs
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        sensor_msgs
        std_msgs
)
