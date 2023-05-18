#ifndef __PARKING_H__
#define __PARKING_H__

#include <itri_msgs/WaypointArray.h>
#include <vector>

float GetAngleVariation(float startAngle, float endAngle);

float GapToParkingMainPath(
    const itri_msgs::WaypointArray & waypoints, const float carRearAxleX,
    const float carRearAxleY, const int indexWaypoint);

double ParkingSteerReference(const int indexWaypoint, float carHead,
    const itri_msgs::WaypointArray & waypoints);

float SteeringCommandForParking(
    const itri_msgs::WaypointArray & waypoints,
    const float carX, const float carY,
    const float carHead, const double gearRatio, const double steerOffset);

float GetRadOfCurvature(
    const float & prevX, const float & prevY,
    const float & targetX, const float & targetY,
    const float & nextX, const float & nextY);

#endif // __PARKING_H__
