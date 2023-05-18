#include <gtest/gtest.h>
#include <file_loader.h>
#include <waypoint_follower.h>

static const float SMALL_VALUE = 0.01f;

TEST(WaypointFollower, AngleErrorToWaypointStraightPath)
{
    itri_msgs::WaypointArray waypoints;
    waypoints.waypoints.resize(50);
    for (size_t i = 0; i < waypoints.waypoints.size(); i ++)
    {
        waypoints.waypoints[i].pose.pose.position.x = static_cast<float>(i);
        waypoints.waypoints[i].pose.pose.position.y = 0.0f;
    }

    int indexMatch = DoPathMatching(3.5f, 1.0f, waypoints);
    float angleErr = AngleErrorToWaypoint(
        3.5f, 1.0f, 0.0f, 2.0f, indexMatch, waypoints, 1.0f, 1.0f);
    ASSERT_NEAR(angleErr, -0.2783f, SMALL_VALUE);

    indexMatch = DoPathMatching(3.2f, -1.0f, waypoints);
    angleErr = AngleErrorToWaypoint(
        3.2f, -1.0f, 0.0f, 2.0f, indexMatch, waypoints, 1.0f, 1.0f);
    ASSERT_NEAR(angleErr, 0.2573f, SMALL_VALUE);

    indexMatch = DoPathMatching(3.2f, -1.0f, waypoints);
    angleErr = AngleErrorToWaypoint(
        3.2f, -1.0f, 0.3f, 2.0f, indexMatch, waypoints, 1.0f, 1.0f);
    ASSERT_NEAR(angleErr, -0.04268f, SMALL_VALUE);
}

TEST(WaypointFollower, AngleErrorToWaypointCurvePathLowResolution)
{
    std::vector<float> pathX;
    std::vector<float> pathY;
    LoadPathData("path/low_resolution_path.json", pathX, pathY);

    itri_msgs::WaypointArray waypoints;
    waypoints.waypoints.resize(pathX.size());
    for (size_t i = 0; i < waypoints.waypoints.size(); i ++)
    {
        waypoints.waypoints[i].pose.pose.position.x = pathX[i];
        waypoints.waypoints[i].pose.pose.position.y = pathY[i];
    }

    int indexMatch = DoPathMatching(158.0f, -29.0f, waypoints);
    float angleErr = AngleErrorToWaypoint(
        158.0f, -29.0f, 2.425f, 2.0f, indexMatch, waypoints, 1.0f, 1.0f);
    ASSERT_NEAR(angleErr, 0.080965f, SMALL_VALUE);

    indexMatch = DoPathMatching(141.0f, -32.0f, waypoints);
    angleErr = AngleErrorToWaypoint(
        141.0f, -32.0f, -1.692f, 2.0f, indexMatch, waypoints, 1.0f, 1.0f);
    ASSERT_NEAR(angleErr, 0.2356f, SMALL_VALUE);
}

TEST(WaypointFollower, AngleErrorToWaypointCurvePathHighResolution)
{
    std::vector<float> pathX;
    std::vector<float> pathY;
    LoadPathData("path/high_resolution_path.json", pathX, pathY);

    itri_msgs::WaypointArray waypoints;
    waypoints.waypoints.resize(pathX.size());
    for (size_t i = 0; i < waypoints.waypoints.size(); i ++)
    {
        waypoints.waypoints[i].pose.pose.position.x = pathX[i];
        waypoints.waypoints[i].pose.pose.position.y = pathY[i];
    }

    int indexMatch = DoPathMatching(158.0f, -29.0f, waypoints);
    float angleErr = AngleErrorToWaypoint(
        158.0f, -29.0f, 2.425f, 2.0f, indexMatch, waypoints, 1.0f, 0.1f);
    ASSERT_NEAR(angleErr, 0.07876f, SMALL_VALUE);

    indexMatch = DoPathMatching(141.0f, -32.0f, waypoints);
    angleErr = AngleErrorToWaypoint(
        141.0f, -32.0f, -1.692f, 2.0f, indexMatch, waypoints, 1.0f, 0.1f);
    ASSERT_NEAR(angleErr, 0.2408f, SMALL_VALUE);
}

TEST(WaypointFollower, AngleErrorToWaypointLargeAngleDifference)
{
    itri_msgs::WaypointArray waypoints;
    waypoints.waypoints.resize(50);
    for (size_t i = 0; i < waypoints.waypoints.size(); i ++)
    {
        waypoints.waypoints[i].pose.pose.position.x = static_cast<float>(i);
        waypoints.waypoints[i].pose.pose.position.y = 0.0f;
    }

    int indexMatch = DoPathMatching(4.0f, 0.0f, waypoints);
    float angleErr = AngleErrorToWaypoint(
        4.0f, 0.0f, 5.89f, 2.0f, indexMatch, waypoints, 1.0f, 1.0f);
    ASSERT_NEAR(angleErr, 0.39319f, SMALL_VALUE);

    angleErr = AngleErrorToWaypoint(
        4.0f, 0.0f, -5.89f, 2.0f, indexMatch, waypoints, 1.0f, 1.0f);
    ASSERT_NEAR(angleErr, -0.39319f, SMALL_VALUE);
}

TEST(WaypointFollower, ConvertToSteeringWheelAngle)
{
    float steeringAngle = ConvertToSteeringWheelAngle(0.1f, 19.6f, 0.0f);
    ASSERT_NEAR(steeringAngle, -112.3f, SMALL_VALUE);

    steeringAngle = ConvertToSteeringWheelAngle(-0.2f, 19.6f, 0.0f);
    ASSERT_NEAR(steeringAngle, 224.6f, SMALL_VALUE);
}

TEST(WaypointFollower, DoPathMatching)
{
    itri_msgs::WaypointArray waypoints;
    waypoints.waypoints.resize(4);
    for (size_t i = 0; i < waypoints.waypoints.size(); i ++)
    {
        waypoints.waypoints[i].pose.pose.position.x = static_cast<float>(i);
        waypoints.waypoints[i].pose.pose.position.y = static_cast<float>(i);
    }

    int indexMatch = DoPathMatching(2.5f, 1.5f, waypoints);
    ASSERT_EQ(indexMatch, 2);

    indexMatch = DoPathMatching(1.5f, 4.0f, waypoints);
    ASSERT_EQ(indexMatch, 3);
}

TEST(WaypointFollower, LateralErrorToWaypoint)
{
    std::vector<float> pathX;
    std::vector<float> pathY;
    LoadPathData("path/high_resolution_path.json", pathX, pathY);

    itri_msgs::WaypointArray waypoints;
    waypoints.waypoints.resize(pathX.size());
    for (size_t i = 0; i < waypoints.waypoints.size(); i ++)
    {
        waypoints.waypoints[i].pose.pose.position.x = pathX[i];
        waypoints.waypoints[i].pose.pose.position.y = pathY[i];
        if (i == waypoints.waypoints.size() - 1)
            waypoints.waypoints[i].pose.pose.orientation.z =
                waypoints.waypoints[i-1].pose.pose.orientation.z;
        else
            waypoints.waypoints[i].pose.pose.orientation.z =
                std::atan2(pathY[i + 1] - pathY[i], pathX[i + 1] - pathX[i]);
    }

    int indexMatch = DoPathMatching(158.0f, -29.0f, waypoints);
    float lateralErr = LateralErrorToWaypoint(
        158.0f, -29.0f, 2.425f, waypoints.waypoints[indexMatch]);
    ASSERT_NEAR(lateralErr, -0.34095f, SMALL_VALUE);

    indexMatch = DoPathMatching(141.0f, -32.0f, waypoints);
    lateralErr = LateralErrorToWaypoint(
        141.0f, -32.0f, -1.692f, waypoints.waypoints[indexMatch]);
    ASSERT_NEAR(lateralErr, -0.26283f, SMALL_VALUE);
}

TEST(WaypointFollower, YawEffect)
{
    float yawRateEffect = YawRateEffect(0.0f, 0.01f, 1.5f);
    ASSERT_NEAR(yawRateEffect, 0.0075f, SMALL_VALUE);

    yawRateEffect = YawRateEffect(0.0f, -0.02f, 1.5f);
    ASSERT_NEAR(yawRateEffect, -0.015f, SMALL_VALUE);

    yawRateEffect = YawRateEffect(0.1f, -0.02f, 1.5f);
    ASSERT_NEAR(yawRateEffect, -0.009f, SMALL_VALUE);
}
