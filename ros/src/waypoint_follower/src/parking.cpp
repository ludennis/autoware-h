#include <cmath>
#include <waypoint_follower.h>

static const float REAR_WHEEL_BASE = 1.112f;
static const float PARKING_STEER_GAIN = 60.0f;
static const float PARKING_STEER_OFFSET = 35.14f;
static const float RADIUS_OFFSET_MULTIPLY_FACTOR = 173.943;

template <typename T>
static inline T Clamp(const T & value, T bottom, T top)
{
    return std::max(bottom, std::min(top, value));
}

static inline float GetPrincipalAngle(const float angle)
{
    float angleMod = std::fmod(angle, 2.0f * M_PI);
    if (std::signbit(angleMod))
        angleMod += 2.0f * M_PI;
    return angleMod;
}

float GetAngleVariation(float startAngle, float endAngle)
{
    float angleVariation = GetPrincipalAngle(endAngle - startAngle);

    if (angleVariation > M_PI)
    {
        angleVariation -= 2 * M_PI;
    }

    return angleVariation;
}

float GapToParkingMainPath(
    const itri_msgs::WaypointArray & waypoints,
    const float carRearAxleX,
    const float carRearAxleY,
    const int indexWaypoint)
{
    const float gapX =
        carRearAxleX - waypoints.waypoints[indexWaypoint].pose.pose.position.x;
    const float gapY =
        carRearAxleY - waypoints.waypoints[indexWaypoint].pose.pose.position.y;

    const float gapToPath =
        std::sin(waypoints.waypoints[indexWaypoint].pose.pose.orientation.z) * gapX -
        std::cos(waypoints.waypoints[indexWaypoint].pose.pose.orientation.z) * gapY;

    return gapToPath;
}

float GetRadOfCurvature(
    const float & prevX, const float & prevY,
    const float & targetX, const float & targetY,
    const float & nextX, const float & nextY)
{
    const double minConst = 0.000000005;
    float sideA = std::hypot(targetX - prevX, targetY - prevY);
    float sideB = std::hypot(nextX - prevX, nextY - prevY);
    float sideC = std::hypot(nextX - targetX, nextY - targetY);
    double s = (sideA + sideB + sideC) / 2;
    if ((s - sideA == 0) || (s - sideB == 0) || (s - sideC == 0))
    {
        s += minConst;
    }
    float triangleArea =
        std::sqrt(std::fabs(s * (s - sideA) * (s - sideB) * (s - sideC)));
    float Radius = (sideA * sideB * sideC) / (4 * triangleArea);
    return Radius;
}

double ParkingSteerReference(const int indexWaypoint, float carHead,
    const itri_msgs::WaypointArray & waypoints)
{
    carHead = GetPrincipalAngle(carHead + M_PI);

    int index = indexWaypoint + 10 > waypoints.waypoints.size() - 1?
        waypoints.waypoints.size() - 1 : indexWaypoint + 10;

    float pathHeadVariation = GetAngleVariation(
        carHead, waypoints.waypoints[index].pose.pose.orientation.z);

    float radius;
    if(indexWaypoint == waypoints.waypoints.size() - 1)
    {
        radius= GetRadOfCurvature(
        waypoints.waypoints[indexWaypoint - 2].pose.pose.position.x,
        waypoints.waypoints[indexWaypoint - 2].pose.pose.position.y,
        waypoints.waypoints[indexWaypoint - 1].pose.pose.position.x,
        waypoints.waypoints[indexWaypoint - 1].pose.pose.position.y,
        waypoints.waypoints[indexWaypoint].pose.pose.position.x,
        waypoints.waypoints[indexWaypoint].pose.pose.position.y);
    }
    else if(indexWaypoint == 0)
    {
        radius= GetRadOfCurvature(
        waypoints.waypoints[indexWaypoint].pose.pose.position.x,
        waypoints.waypoints[indexWaypoint].pose.pose.position.y,
        waypoints.waypoints[indexWaypoint + 1].pose.pose.position.x,
        waypoints.waypoints[indexWaypoint + 1].pose.pose.position.y,
        waypoints.waypoints[indexWaypoint + 2].pose.pose.position.x,
        waypoints.waypoints[indexWaypoint + 2].pose.pose.position.y);
    }
    else
    {
        radius= GetRadOfCurvature(
        waypoints.waypoints[indexWaypoint - 1].pose.pose.position.x,
        waypoints.waypoints[indexWaypoint - 1].pose.pose.position.y,
        waypoints.waypoints[indexWaypoint].pose.pose.position.x,
        waypoints.waypoints[indexWaypoint].pose.pose.position.y,
        waypoints.waypoints[indexWaypoint + 1].pose.pose.position.x,
        waypoints.waypoints[indexWaypoint + 1].pose.pose.position.y);
    }

    double parkingSteerOffset = RADIUS_OFFSET_MULTIPLY_FACTOR / radius ;

    double ParkingSteerReference;

    if (std::abs(pathHeadVariation)< 0.03f)
        ParkingSteerReference = 0.0f;
    else
    {
        if(std::signbit(pathHeadVariation))
            ParkingSteerReference = -parkingSteerOffset;
        else
            ParkingSteerReference = parkingSteerOffset;
    }

    return ParkingSteerReference;
}


float SteeringCommandForParking(
    const itri_msgs::WaypointArray & waypoints,
    const float carX,
    const float carY,
    const float carHead,
    const double gearRatio,
    const double steerOffset)
{
    float carReverseHead = GetPrincipalAngle(carHead + M_PI);
    float carRearAxleX = carX + std::cos(carReverseHead) * REAR_WHEEL_BASE;
    float carRearAxleY = carY + std::sin(carReverseHead) * REAR_WHEEL_BASE;

    const int indexWaypoint =
        DoPathMatching(carRearAxleX, carRearAxleY, waypoints);

    double parkingSteerReference = ParkingSteerReference(
        indexWaypoint, carHead, waypoints);

    // calaulate car location, positive for path right side
    float gapToPath = GapToParkingMainPath(
        waypoints, carRearAxleX, carRearAxleY, indexWaypoint);

    float steeringCmd =
        gearRatio * (parkingSteerReference + PARKING_STEER_GAIN * gapToPath);
    steeringCmd += static_cast<float>(steerOffset);
    return Clamp(steeringCmd, -720.0f, 720.0f);
}
