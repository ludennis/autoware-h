#include <file_loader.h>
#include <gtest/gtest.h>
#include <motion_planner/parking_utils.h>


static const float SMALL_VALUE = 1.0e-03f;

static inline float PrincipleAngle(const float angle)
{
    float anglePrcp = std::atan2(std::sin(angle), std::cos(angle));
    if (std::signbit(anglePrcp))
        anglePrcp += 2.0f * M_PI;
    return anglePrcp;
}

TEST(ParkingPlannerUtils, GetProjectionPoint)
{
    /*=========== case1 ===========*/
    itri_msgs::Waypoint p;
    p.pose.pose.position.x = 3.0f;
    p.pose.pose.position.y = 4.0f;
    itri_msgs::Waypoint l1;
    l1.pose.pose.position.x = 10.0f;
    l1.pose.pose.position.y = 0.0f;
    itri_msgs::Waypoint l2;
    l2.pose.pose.position.x = 11.0f;
    l2.pose.pose.position.y = 0.0f;

    itri_msgs::Waypoint projectionPoint;
    ParkingPlannerUtils::GetProjectionPoint(p,l1,l2,projectionPoint);
    ASSERT_EQ(3.0f, projectionPoint.pose.pose.position.x);
    ASSERT_EQ(0.0f, projectionPoint.pose.pose.position.y);

    /*=========== case2 ===========*/
    p.pose.pose.position.x = 0.0f;
    p.pose.pose.position.y = 0.0f;
    l1.pose.pose.position.x = 1.0f;
    l1.pose.pose.position.y = 0.0f;
    l2.pose.pose.position.x = 0.0f;
    l2.pose.pose.position.y = 1.0f;

    ParkingPlannerUtils::GetProjectionPoint(p,l1,l2,projectionPoint);
    ASSERT_EQ(0.5f, projectionPoint.pose.pose.position.x);
    ASSERT_EQ(0.5f, projectionPoint.pose.pose.position.y);
}

TEST(ParkingPlannerUtils, StraightPathGen)
{
    //=================== Case 1 ======================
    itri_msgs::Waypoint targetPoint;
    targetPoint.pose.pose.position.x = 113.0;
    targetPoint.pose.pose.position.y = -67.0;
    itri_msgs::Waypoint startPoint;
    startPoint.pose.pose.position.x = 117.0;
    startPoint.pose.pose.position.y = -66.0;
    float resolution = 0.1f;

    itri_msgs::WaypointArray tPath;
    ParkingPlannerUtils::StraightPathGen(resolution, targetPoint.pose.pose,
        startPoint.pose.pose, tPath);
    itri_msgs::Waypoint pathEnd = tPath.waypoints.back();

    // check lenghth
    float distance2TargetPoint = std::sqrt(17.0f);

    float pathLength = std::hypot(
        tPath.waypoints[tPath.waypoints.size()-1].pose.pose.position.x-
            tPath.waypoints[0].pose.pose.position.x,
        tPath.waypoints[tPath.waypoints.size()-1].pose.pose.position.y-
            tPath.waypoints[0].pose.pose.position.y);
    ASSERT_NEAR(distance2TargetPoint, pathLength, resolution);


    // check pathend
    float targetPointError = std::hypot(
        targetPoint.pose.pose.position.x - pathEnd.pose.pose.position.x,
        targetPoint.pose.pose.position.y - pathEnd.pose.pose.position.y);
    ASSERT_NEAR(targetPointError, 0, std::sqrt(2*resolution*resolution));

    //check path direction
    float pathHead = PrincipleAngle(std::atan2
        (targetPoint.pose.pose.position.y - startPoint.pose.pose.position.y,
        targetPoint.pose.pose.position.x - startPoint.pose.pose.position.x));
    ASSERT_NEAR(
        pathHead,tPath.waypoints[tPath.waypoints.size()-1].pose.pose.orientation.z,0.1f);

    //=================== Case 2 ======================
    targetPoint.pose.pose.position.x = -58.0;
    targetPoint.pose.pose.position.y = -67.0;
    startPoint.pose.pose.position.x = -62.0;
    startPoint.pose.pose.position.y = -71.0;

    itri_msgs::WaypointArray t2Path;

    ParkingPlannerUtils::StraightPathGen(resolution, targetPoint.pose.pose,
        startPoint.pose.pose, t2Path);
    pathEnd = t2Path.waypoints.back();

    distance2TargetPoint = std::sqrt(32.0f);

    pathLength = std::hypot(
        t2Path.waypoints[t2Path.waypoints.size()-1].pose.pose.position.x-
            t2Path.waypoints[0].pose.pose.position.x,
        t2Path.waypoints[t2Path.waypoints.size()-1].pose.pose.position.y-
            t2Path.waypoints[0].pose.pose.position.y);
    ASSERT_NEAR(distance2TargetPoint, pathLength, resolution);


    // check pathend
    targetPointError = std::hypot(
        targetPoint.pose.pose.position.x - pathEnd.pose.pose.position.x,
        targetPoint.pose.pose.position.y - pathEnd.pose.pose.position.y);
    ASSERT_NEAR(targetPointError, 0, std::sqrt(2*resolution*resolution));

    //check path direction
    pathHead = PrincipleAngle(std::atan2
        (targetPoint.pose.pose.position.y - startPoint.pose.pose.position.y,
        targetPoint.pose.pose.position.x - startPoint.pose.pose.position.x));
    ASSERT_NEAR(
        pathHead,t2Path.waypoints[t2Path.waypoints.size()-1].pose.pose.orientation.z,0.1f);
}

TEST(ParkingPlannerUtils, ArcGen)
{
    itri_msgs::WaypointArray gpathXY;
    float diffX = 0.1f * std::cos(M_PI / 6.0);
    float diffY = 0.1f * std::sin(M_PI / 6.0);

    for(int i = 0; i < 20.0 / 0.1; i++)
    {
        itri_msgs::Waypoint tmpPathXY;
        tmpPathXY.pose.pose.position.x = diffX * i;
        tmpPathXY.pose.pose.position.y = diffY * i;
        tmpPathXY.pose.pose.orientation.z = 7 * M_PI / 6.0;

        gpathXY.waypoints.push_back(tmpPathXY);
    }
    std::reverse(gpathXY.waypoints.begin(), gpathXY.waypoints.end());

    //=================== Case 1 ======================

    itri_msgs::WaypointArray parkingSpaceXY;
    itri_msgs::Waypoint parkingCorner;
    parkingCorner.pose.pose.position.x = 11.0f;
    parkingCorner.pose.pose.position.y = 8.0f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 9.0f;
    parkingCorner.pose.pose.position.y = 8.0f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 11.0f;
    parkingCorner.pose.pose.position.y = 12.0f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 9.0f;
    parkingCorner.pose.pose.position.y = 12.0f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);

    ParkingPlannerUtils::ParkingSpaceInfo parkingSpaceInfo;
    ParkingPlannerUtils::ParkingSpaceInfoUpdate(
        parkingSpaceXY, gpathXY, parkingSpaceInfo);


    float parkingHead = - M_PI_2;
    int direction = -1;
    itri_msgs::WaypointArray parkingCurvePath;

    itri_msgs::Waypoint pathXY;
    pathXY.pose.pose.position.x = 10.0f;
    pathXY.pose.pose.position.y = 10.0f;
    pathXY.pose.pose.orientation.z = M_PI_2;

    float diffTheta = 0.02f;
    float diffLength = 0.1f;
    float angleVariation = M_PI / 3 / diffTheta;

    parkingCurvePath.waypoints.push_back(pathXY);

    ParkingPlannerUtils::ArcGen(angleVariation,
        parkingSpaceInfo.mSearchPathHead, parkingSpaceInfo.mParkingLocation,
        direction, diffTheta, diffLength, parkingCurvePath, pathXY);

    // check arc end position
    ASSERT_NEAR(7.444f, parkingCurvePath.waypoints.back().pose.pose.position.x,0.1f);
    ASSERT_NEAR(5.638f, parkingCurvePath.waypoints.back().pose.pose.position.y,0.1f);
    // check arc end direction
    ASSERT_NEAR(M_PI / 2, parkingCurvePath.waypoints.back().pose.pose.orientation.z, 0.1f);

    //=================== Case 2 ======================
    itri_msgs::WaypointArray parkingSpaceXY2;

    parkingCorner.pose.pose.position.x = 19.0f;
    parkingCorner.pose.pose.position.y = 1.732f;
    parkingSpaceXY2.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 10.34f;
    parkingCorner.pose.pose.position.y = -3.268f;
    parkingSpaceXY2.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 21.0f;
    parkingCorner.pose.pose.position.y = -1.732f;
    parkingSpaceXY2.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 12.38f;
    parkingCorner.pose.pose.position.y = -6.732f;
    parkingSpaceXY2.waypoints.push_back(parkingCorner);

    ParkingPlannerUtils::ParkingSpaceInfoUpdate(
        parkingSpaceXY, gpathXY, parkingSpaceInfo);


    parkingHead = 7 * M_PI / 6;
    direction = 1;
    itri_msgs::WaypointArray parkingCurvePath2;

    pathXY.pose.pose.position.x = 20.0f;
    pathXY.pose.pose.position.y = 0.0f;
    pathXY.pose.pose.orientation.z = PrincipleAngle(parkingHead + M_PI);
    angleVariation = M_PI / 6 / diffTheta;

    parkingCurvePath2.waypoints.push_back(pathXY);

    ParkingPlannerUtils::ArcGen(angleVariation,
        parkingSpaceInfo.mSearchPathHead, parkingSpaceInfo.mParkingLocation,
        direction, diffTheta, diffLength,parkingCurvePath, pathXY);

    // check arc end position
    ASSERT_NEAR(20.0f, parkingCurvePath2.waypoints.back().pose.pose.position.x,0.1f);
    ASSERT_NEAR(0.0f, parkingCurvePath2.waypoints.back().pose.pose.position.y,0.1f);
    // check arc end direction
    ASSERT_NEAR(M_PI / 6, parkingCurvePath2.waypoints.back().pose.pose.orientation.z, 0.1f);
}
