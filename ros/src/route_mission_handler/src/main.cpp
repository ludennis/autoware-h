#include <ros/ros.h>
#include <route_mission_handler/route_data_handler.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_data_handler");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");
    LaneConstructor laneConstructor(node, privateNode);

    ros::Subscriber waypointPathSub = node.subscribe(
        "/global_waypoints_display",
        10,
        &LaneConstructor::WaypointsCallback,
        &laneConstructor);

    ros::Subscriber navgRoadSub = node.subscribe(
        "/global_navgroads_display",
        10,
        &LaneConstructor::NavgRoadsCallback,
        &laneConstructor);

    ros::Subscriber laneNavgRoadSub = node.subscribe(
        "/global_lanenavgroads_display",
        10,
        &LaneConstructor::LaneNavgRoadsCallback,
        &laneConstructor);

    ros::Subscriber roadLineSub = node.subscribe(
        "/road_lines_display",
        10,
        &LaneConstructor::RoadLinesCallback,
        &laneConstructor);
    ros::Subscriber roadMarkerSub = node.subscribe(
        "/road_markers_display",
        10,
        &LaneConstructor::RoadMarkersCallback,
        &laneConstructor);

    ros::Subscriber laneInfoSub = node.subscribe(
        "/laneinfo_fromfile",
        10,
        &LaneConstructor::LaneInfoCallback,
        &laneConstructor);
    ros::Subscriber nonAccessibleSub = node.subscribe(
        "/non_accessibles",
        10,
        &LaneConstructor::NonAccessiblesCallback,
        &laneConstructor);
    ros::Subscriber parkingSpaceSub = node.subscribe(
        "/parking_spaces",
        10,
        &LaneConstructor::ParkingSpacesCallback,
        &laneConstructor);

    ros::Subscriber trafficLightSub = node.subscribe(
        "/traffic_lights",
        10,
        &LaneConstructor::TrafficLightCallback,
        &laneConstructor);

    ros::Subscriber loadCompletedSub = node.subscribe(
        "/load_completed",
        10,
        &LaneConstructor::LoadCompletedCallback,
        &laneConstructor);

    ros::spin();

    return 0;
}
