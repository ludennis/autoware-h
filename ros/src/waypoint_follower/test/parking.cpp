#include <cmath>
#include <cstring>
#include <gtest/gtest.h>

TEST(SteeringControlForParking, GetAngleVariation)
{
    float startAngle = 30.0 * M_PI / 180.0f;
    float endAngle = 330.0 * M_PI / 180.0f;
    float AngleVariation = GetAngleVariation(startAngle, endAngle);
    ASSERT_LT(AngleVariation, 0.0f);
    ASSERT_NEAR(AngleVariation, -60.0f* M_PI / 180.0f, 0.001f);

    startAngle = 30.0 * M_PI / 180.0f;
    endAngle = 70.0 * M_PI / 180.0f;
    AngleVariation = GetAngleVariation(startAngle, endAngle);
    ASSERT_GT(AngleVariation, 0.0f);
    ASSERT_NEAR(AngleVariation, 40.0f* M_PI / 180.0f, 0.001f);

    startAngle = 230.0 * M_PI / 180.0f;
    endAngle = 150.0 * M_PI / 180.0f;
    AngleVariation = GetAngleVariation(startAngle, endAngle);
    ASSERT_LT(AngleVariation, 0.0f);
    ASSERT_NEAR(AngleVariation, -80.0f* M_PI / 180.0f, 0.001f);

    startAngle = 260.0 * M_PI / 180.0f;
    endAngle = 10.0 * M_PI / 180.0f;
    AngleVariation = GetAngleVariation(startAngle, endAngle);
    ASSERT_GT(AngleVariation, 0.0f);
    ASSERT_NEAR(AngleVariation, 110.0f* M_PI / 180.0f, 0.001f);

    startAngle = 10.0 * M_PI / 180.0f;
    endAngle = 210.0 * M_PI / 180.0f;
    AngleVariation = GetAngleVariation(startAngle, endAngle);
    ASSERT_LT(AngleVariation, 0.0f);
    ASSERT_NEAR(AngleVariation, -160.0f* M_PI / 180.0f, 0.001f);

    startAngle = 350.0 * M_PI / 180.0f;
    endAngle = 140.0 * M_PI / 180.0f;
    AngleVariation = GetAngleVariation(startAngle, endAngle);
    ASSERT_GT(AngleVariation, 0.0f);
    ASSERT_NEAR(AngleVariation, 150.0f* M_PI / 180.0f, 0.001f);
}

TEST(SteeringControlForParking, CalaulateGapToParkingMainPath)
{
    itri_msgs::WaypointArray waypoints;
    waypoints.waypoints.resize(5);
    for (size_t i = 0; i < waypoints.waypoints.size(); i ++)
    {
        waypoints.waypoints[i].pose.pose.position.x = static_cast<float>(i);
        waypoints.waypoints[i].pose.pose.position.y = static_cast<float>(i);
        waypoints.waypoints[i].pose.pose.orientation.z = M_PI_4;
    }

    float gap = GapToParkingMainPath(
        waypoints, 3.0f, 1.0f, 2);
    ASSERT_NEAR(gap, std::sqrt(2.0f), 0.001f);

    gap = GapToParkingMainPath(
        waypoints, 1.0f, 3.0f, 2);
    ASSERT_NEAR(gap, -std::sqrt(2.0f), 0.001f);
}

TEST(SteeringControlForParking, ParkingSteerReference)
{
    itri_msgs::WaypointArray waypoints;
    waypoints.waypoints.resize(11);
    for (size_t i = 0; i < waypoints.waypoints.size(); i ++)
    {
        waypoints.waypoints[i].pose.pose.orientation.z =
            M_PI_4 * static_cast<float>(i) / 10.0;
        waypoints.waypoints[i].pose.pose.position.x =
            (40 / M_PI) * std::sin(i * M_PI / 40);
        waypoints.waypoints[i].pose.pose.position.y =
            (40 / M_PI) * (1 - std::cos(i * M_PI / 40));
    }

    double parkingSteerReference = ParkingSteerReference(
        4, M_PI * 1.1, waypoints);
    ASSERT_NEAR(parkingSteerReference, 13.6614f, 0.001f);

    for (auto & waypoint : waypoints.waypoints)
    {
        waypoint.pose.pose.orientation.z *= -1.0;
        waypoint.pose.pose.position.y *= -1.0;
    }
    parkingSteerReference = ParkingSteerReference(
        4, M_PI * 0.9, waypoints);
    ASSERT_NEAR(parkingSteerReference, -13.6614f, 0.001f);
}

TEST(SteeringControlForParking, GetRadOfCurvature)
{
    /*=== CCW ===*/
    float prevX = 0.0f;
    float prevY = -4.0f;
    float targetX = 4.0f / std::sqrt(2);
    float targetY = -4.0f / std::sqrt(2);
    float nextX = 4.0f;
    float nextY = 0.0f;

    float turn_radius = GetRadOfCurvature(prevX, prevY, targetX, targetY,
        nextX, nextY);
    ASSERT_NEAR(turn_radius, 4.0f, 0.001f);

    /*=== CW ===*/
    prevX = 0.0f;
    prevY = -3.0f;
    targetX = -3.0f / std::sqrt(2);
    targetY = -3.0f / std::sqrt(2);
    nextX = -3.0f;
    nextY = 0.0f;

    turn_radius = GetRadOfCurvature(prevX, prevY, targetX, targetY,
        nextX, nextY);
    ASSERT_NEAR(turn_radius, 3.0f, 0.001f);
}
