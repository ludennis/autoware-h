#ifndef __WAYPOINT_FOLLOWER_H__
#define __WAYPOINT_FOLLOWER_H__

#include <itri_msgs/WaypointArray.h>
#include <vector>

float AngleErrorToWaypoint(
    const float carX, const float carY, const float carHead,
    const float carSpeed, const int indexWaypoint,
    const itri_msgs::WaypointArray & waypoints,
    const float predictAheadTime, const float wayPtResolution);

float ConvertToSteeringWheelAngle(
    const float steeringCmd, const float gearRatio, const float steerOffset);

int DoPathMatching(
    const float x, const float y,
    const itri_msgs::WaypointArray & waypoints);

float LateralErrorToWaypoint(
    const float carX, const float carY, const float carHead,
    const itri_msgs::Waypoint & waypoint);

float YawRateEffect(
    const float angleError, const float carYawRate,
    const float predictAheadTime);

#endif // __WAYPOINT_FOLLOWER_H__
