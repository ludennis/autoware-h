#ifndef _LANEINFO_BROADCAST_UTIL_H_
#define _LANEINFO_BROADCAST_UTIL_H_

#include <ros/ros.h>
#include <behavior/lane_type.h>
#include <itri_msgs/Waypoint.h>

const float GPS_RANGE = 3.0;

void UpdateDetectRange(float speed, float &vehicleSpeed);

float GetDetectRange();

bool GetNearPointIndex(
	std::vector<itri_msgs::Waypoint> &points,
    itri_msgs::Waypoint &vehicleWaypoint,
    int &nearIndex);

int GetSearchIndex(
	int searchIndex,
    std::vector<itri_msgs::Waypoint> &waypoints);

float GetDistanceBetweenToPoints(float x1, float y1, float x2, float y2);

int MappingToEnumRoadsignType(std::string &type);

int MappingToEnumLaneType(std::string &type);

#endif