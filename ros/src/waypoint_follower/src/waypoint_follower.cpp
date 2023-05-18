#include <algorithm>
#include <cmath>
#include <iostream>
#include <trace/utils.h>
#include <waypoint_follower.h>

#define TRACE_TAG "WaypointFollower"

static const float BASE_DISTANCE = 3.5f;
static const float DISTANCE_LIMIT = 30.0f;
static const float RADIUS_DEGREE_RATIO = 180.0f / M_PI;
static const float STEERING_ANGLE_LIMIT = 720.0f;
static const float YAW_RATE_EFFECT_GAIN_MAX = 0.6f;
static const float YAW_RATE_EFFECT_GAIN_MIN = 0.3f;
static const float DELAY_ANGLE = 0.05f;
static const float YAW_GAIN_DECAY_ANGLE = 0.02f;
static const float YAW_GAIN_BASE = 1.0f;

template <typename T>
static inline T Clamp(const T & value, T bottom, T top)
{
    return std::max(bottom, std::min(top, value));
}

static inline float RadiusToDegree(const float radius)
{
    return RADIUS_DEGREE_RATIO * radius;
}

static inline float PrincipleAngleMinusPi(const float angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

float AngleErrorToWaypoint(
    const float carX, const float carY, const float carHead,
    const float carSpeed, const int indexWaypoint,
    const itri_msgs::WaypointArray & waypoints,
    const float predictAheadTime, const float wayPtResolution)
{
    const float lookAheadDistance = Clamp(
        predictAheadTime * carSpeed, BASE_DISTANCE, DISTANCE_LIMIT);
    int indexAhead =
        indexWaypoint + static_cast<int>(lookAheadDistance / wayPtResolution);
    indexAhead = Clamp(
        indexAhead, 0, static_cast<int>(waypoints.waypoints.size()) - 1);

    const float deltaTheta = std::atan2(
        waypoints.waypoints[indexAhead].pose.pose.position.y - carY,
        waypoints.waypoints[indexAhead].pose.pose.position.x - carX);

    return PrincipleAngleMinusPi(deltaTheta - carHead);
}

float ConvertToSteeringWheelAngle(
    const float steeringCmd, const float gearRatio, const float steerOffset)
{
    const float steeringWheelAngle =
        -steeringCmd * gearRatio + steerOffset;
    return Clamp(RadiusToDegree(
        steeringWheelAngle), -STEERING_ANGLE_LIMIT, STEERING_ANGLE_LIMIT);
}

int DoPathMatching(
    const float x, const float y,
    const itri_msgs::WaypointArray & waypoints)
{
    TRACE_ASSERT_THROW(waypoints.waypoints.size() > 0);

    std::vector<float> distance;
    for (auto const & waypoint : waypoints.waypoints)
    {
        const float diffX = waypoint.pose.pose.position.x - x;
        const float diffY = waypoint.pose.pose.position.y - y;
        distance.push_back(std::hypot(diffX, diffY));
    }
    int indexMatch =
        std::min_element(distance.begin(), distance.end()) - distance.begin();

    return indexMatch;
}

float LateralErrorToWaypoint(
    const float carX, const float carY, const float carHead,
    const itri_msgs::Waypoint & waypoint)
{
    const float diffX = carX - waypoint.pose.pose.position.x;
    const float diffY = carY - waypoint.pose.pose.position.y;

    const float lateralErr =
        -diffX * std::sin(waypoint.pose.pose.orientation.z) +
        diffY * std::cos(waypoint.pose.pose.orientation.z);

    return lateralErr;
}

float YawRateEffect(
    const float angleError, const float carYawRate,
    const float predictAheadTime)
{
    float gain = YAW_RATE_EFFECT_GAIN_MAX *
        (YAW_GAIN_BASE -
            (std::abs(angleError) - DELAY_ANGLE) / YAW_GAIN_DECAY_ANGLE);
    gain = Clamp(gain, YAW_RATE_EFFECT_GAIN_MIN, YAW_RATE_EFFECT_GAIN_MAX);
    return gain * carYawRate * predictAheadTime;
}
