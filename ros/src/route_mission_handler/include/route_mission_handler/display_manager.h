#ifndef _DISPLAY_MANAGER_HPP_
#define _DISPLAY_MANAGER_HPP_

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <fstream>
#include <string>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <route_mission_handler/RoadLine.h>
#include <route_mission_handler/ParkingSpace.h>
#include <route_mission_handler/TrafficLight.h>
#include <route_mission_handler/MarkerPolygonArray.h>
#include <route_mission_handler/MarkerPolygon.h>
#include <route_mission_handler/Waypoints.h>
#include <route_mission_handler/Lane.h>
#include <route_mission_handler/Croad.h>
#include <route_mission_handler/display_rviz_util.h>
#include <route_mission_handler/utils.h>
#include <geometry_msgs/PoseArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>

using namespace visualization_msgs;

class DisplayManager
{

public:
    DisplayManager(ros::NodeHandle &n);

    void DisplayWaypointMarkers(
        std::map<std::string, route_mission_handler::Waypoints> &poses,
        std::map<int, route_mission_handler::Lane> &lanes,
        std::map<int, std::vector<std::string>> &lanePoints);

    void DisplayNavgRoads(
        std::map<int, route_mission_handler::Croad> &navgRoads);

    void DisplayRoadLines(
        std::map<int, route_mission_handler::RoadLine> &lines);

    void DisplayRoadMarkers(
        std::map<std::string, route_mission_handler::MarkerPolygon> &markers);

    void DisplayParkingSpaces(
        std::map<int, route_mission_handler::ParkingSpace> &spaces);

    void DisplayTrafficLights(
        std::map<int, route_mission_handler::TrafficLight> &lights);

private:

    const float DOUBLE_LINE_INTERVAL = 0.2;

    std::vector<std::string> mMarkerTypes;
    std::map<int, std_msgs::ColorRGBA> mColorsMap;

    ros::Publisher mMarkerArrayPub;
    ros::Publisher mPubRoadMarkers;

    Marker MakeBall(float scale);

    Marker MakeArrow(float scale, geometry_msgs::Pose &pose);

    visualization_msgs::Marker GetLineWithType(
        int index,
        bool isAdd,
        LaneType type);

    visualization_msgs::Marker GetDoubleLineMarker(
        int index,
        bool isAdd,
        LaneType type,
        std::vector<geometry_msgs::Point> &points);

    std::vector<visualization_msgs::Marker> GetLineStripMarker(
        int index,
        bool isAdd,
        std::vector<route_mission_handler::RoadLinePoint> points);
};

#endif
