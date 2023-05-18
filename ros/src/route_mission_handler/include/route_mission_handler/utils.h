#ifndef _COMMON_UTILS_HPP_
#define _COMMON_UTILS_HPP_

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <jsoncpp/json/json.h>
#include <route_mission_handler/MarkerPolygon.h>
#include <route_mission_handler/RoadLine.h>
#include <route_mission_handler/ParkingSpace.h>
#include <route_mission_handler/TrafficLight.h>
#include <route_mission_handler/Waypoint.h>
#include <route_mission_handler/NavgRoad.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <jsoncpp/json/json.h>

using namespace GeographicLib;

extern std::string marker_types;

enum class EditType
{
    NONE,
    WAYPOINT,
    LANE,
    ROAD_MARKER,
    ROAD_LINE,
    NAVG_ROAD, // 5
    INTERSECTION_POINT,
    PARKING_SPACE,
    TRAFFIC_LIGTH,
    NEW_OBJECT
};

struct LaneNavg
{
    bool isPositive1;
    bool isPositive2;

    int navgroad1_id;
    int navgroad2_id;

    int laneId;
    int laneOrder;
    std::string seqner;
};

enum class LaneType {
    EMPTY = 0,
    WHITE_DASHED_LINE,
    WHITE_SOLID_LINE,
    WHITE_DOUBLE_LINE,
    YELLOW_DASHED_LINE,
    YELLOW_SOLID_LINE,
    YELLOW_DOUBLE_LINE, // 6
    RED_SOLID_LINE,
    STRAIGHT_ARROW,
    TURN_LEFT_ARROW,
    TURN_RIGHT_ARROW,
    STRAIGHT_OR_LEFT_ARROW, // 11
    STRAIGHT_OR_RIGHT_ARROW,
    TURN_RIGHT_ONLY,
    STOP_LINE,
    LONGER_BUMP,
    SHORTER_BUMP, // 16
    SLOWDOWN,
    YIELD,
    STOP_SIGN,
    SPEED_LIMIT_20KPH, // 20
    SPEED_LIMIT_30KPH,
    SPEED_LIMIT_50KPH,
    SCOOTER_PARKING_PLACE,
    PARKING_SPACE,
    ZEBRACROSS,
    INTERSECTION_AREA,  // 26
    NON_DRIVING_AREA,
    INTERSECTION_POINT,
    LOW_SPEED,
    LOW_SPEED_LINE, // 30
    YIELD_LINE,
    NO_TEMPPARKING_AREA,
    TEMPPARKING_AREA,
    MAX_SPEED_LIMIT,
    PARKING_WARNING, // 35
    ROAD_WARNING,
    SPEED_WARNING,
    FIRE_HYDRANT,
    ROAD_GUIDE,
    NO_PARKING, //40
    TURNUNG_LINE,
    DRIVING_DRIECTIONS,
    STOP_FOR_INSPECTION,
    WIGHT_RESTRICTION,
    NO_CHANGING_LANES, // 45
    NO_ENTRY,
    PROHIBIT_RIGHT,
    BIKE_LANE,
    SPEED_LIMIT_40KPH,
    TRAFFIC_LIGTH, // 50
    PEDESTRIAN_LIGTH,
    BOUNDARIES,  // 52
    VIRTUAL_LINE,
    TRIPLE_ARROW,
    HSR_STATION
};

bool CheckFileExist(const std::string& filepath);

std::string CreateFile(
    std::string folderPath,
    std::string fileName,
    std::string title);

float GetDegree(float radius);

float GetPrincipalAngle(const float &angle);

float GetDistanceBetweenTwoPoints(
	  float x1,
    float y1,
    float x2,
    float y2);

float GetDistanceBetweenDimensionPoints(
    geometry_msgs::Point &point1,
    geometry_msgs::Point &point2);

std::vector<int> ParseStringToIntArray(
    std::string &dataStr,
    char symbol);

std::vector<std::string> ParseStringToArray(
    std::string &dataStr,
    char symbol);

std::map<int, bool> ParseKeyValues(
    std::string &dataStr,
    char symbol,
    char symbol2,
    std::vector<int> &orders);

void FindSideLanesByValue(
    std::vector<LaneNavg> &vec,
    std::map<int, LaneNavg> mapOfElemen,
    LaneNavg value);

std::string trim(const std::string& str);

void RemoveStringFromVector(std::vector<std::string> &map, std::string value);

void RemoveIntFromVector(std::vector<int> &list, int value);

Json::Value GetRoadMarkerValue(route_mission_handler::MarkerPolygon &marker);

Json::Value GetParkingSpaceValue(
    route_mission_handler::ParkingSpace &parkingSpace);

Json::Value GetTrafficLightValue(route_mission_handler::TrafficLight &light);

Json::Value GetRoadLineValue(route_mission_handler::RoadLine &line);

geometry_msgs::Point TransformLocalCartesian(double lat, double lon);

geometry_msgs::Point TransformLocalCartesian(double x, double y, double z);

int GetNearCroad(
    route_mission_handler::Waypoint waypnt,
    LaneNavg &laneNavg,
    std::map<int, route_mission_handler::NavgRoad> &navgRoadMap);

#endif
