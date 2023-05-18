#ifndef route_data_handler_H
#define route_data_handler_H

#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <route_mission_handler/Path.h>
#include <route_mission_handler/PathsArray.h>
#include <route_mission_handler/LaneNavgRoadArray.h>
#include <route_mission_handler/NavgRoadArray.h>
#include <route_mission_handler/RoadLineArray.h>
#include <route_mission_handler/MarkerPolygonArray.h>
#include <route_mission_handler/ParkingSpaceArray.h>
#include <route_mission_handler/TrafficLightArray.h>
#include <route_mission_handler/LaneArray.h>
#include <route_mission_handler/LanesArray.h>
#include <route_mission_handler/MarkerPolygonArray.h>
#include <route_mission_handler/utils.h>
#include <route_mission_handler/display_manager.h>
#include <ctime>

// #define DEBUG

class LaneConstructor
{

public:

    LaneConstructor(ros::NodeHandle& node, ros::NodeHandle& privateNode);

    void WaypointsCallback(
        const route_mission_handler::PathsArray::ConstPtr &msg);

    void LaneNavgRoadsCallback(
        const route_mission_handler::LaneNavgRoadArray::ConstPtr &msg);

    void NavgRoadsCallback(
        const route_mission_handler::NavgRoadArray::ConstPtr &msg);

    void RoadLinesCallback(
        const route_mission_handler::RoadLineArray::ConstPtr &msg);

    void RoadMarkersCallback(
        const route_mission_handler::MarkerPolygonArray::ConstPtr &msg);

    void LaneInfoCallback(
        const route_mission_handler::LanesArray::ConstPtr &msg);

    void NonAccessiblesCallback(
        const route_mission_handler::MarkerPolygonArray::ConstPtr &msg);

    void ParkingSpacesCallback(
        const route_mission_handler::ParkingSpaceArray::ConstPtr &msg);

    void TrafficLightCallback(
        const route_mission_handler::TrafficLightArray::ConstPtr &msg);

    void LoadCompletedCallback(const std_msgs::Bool::ConstPtr &msg);

private:

    const int twoDigitalThreshold = 10;
    const int threeDigitalThreshold = 100;

    std::string mRoute;
    std::map<int, bool> mRouteNavgcroads;
    std::vector<int> mRouteNavgcroadOrders;
    std::vector<int> mDebugLanes;
    DisplayManager* mDisplayMgr = nullptr;

    std::map<int, route_mission_handler::NavgRoad> mNavgRoadMap;
    std::map<int, route_mission_handler::Lanes> mLanes;
    std::map<int, std::vector<std::string> > mLanePoints;
    std::map<std::string, route_mission_handler::Waypoints> mPointsMap;
    std::map<int, LaneNavg> mLaneNavgMap;
    std::map<int, std::set<std::string> > mLaneMarkers;
    std::map<std::string, route_mission_handler::MarkerPolygon> mRoadMarkerMap;
    std::map<int, route_mission_handler::RoadLine> mRoadLineMap;
    std::map<int, route_mission_handler::ParkingSpace> mParkingSpaces;
    std::map<int, std::vector<int> > mLaneParkingSpaceMap;
    std::map<int, route_mission_handler::TrafficLight> mTrafficLightMap;

    ros::Publisher mPubFinal;

    void UpdateNavgRoadLanes(
        std::map<int, route_mission_handler::NavgRoad> &navgRoads,
        LaneNavg &laneNavg);

    void GetBindingLaneFromNavgRoad(
        route_mission_handler::MarkerPolygon &marker,
        std::vector<int> &laneIds,
        std::map<int, LaneNavg> &laneNavgMap);

    std::vector<route_mission_handler::Lane> GetLanes(
        std::vector<int> &laneIds,
        route_mission_handler::Croad &croad,
        int nextCroadId,
        bool isPositive2,
        std::vector<route_mission_handler::Lane> &lanes);

};

#endif //route_data_handler_H
