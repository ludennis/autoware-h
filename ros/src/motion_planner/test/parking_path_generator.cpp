#include <cmath>
#include <cstring>
#include <file_loader.h>
#include <gtest/gtest.h>
#include <memory>
#include <motion_planner/state.h>
#include <motion_planner/type_id.h>
#include <motion_planner/motion_planner_context.h>
#include <motion_planner/parking_path_generator.h>
#include <ros/init.h>


static const float SMALL_VALUE = 1.0e-03f;

static inline float GetPrincipalAngle(const float & angle)
{
    float angleMod = std::fmod(angle, 2.0f * M_PI);
    if (std::signbit(angleMod))
        angleMod += 2.0f * M_PI;
    return angleMod;
}

inline float GetAngleVariation(float startAngle, float endAngle)
{
    float angleVariation = GetPrincipalAngle(endAngle - startAngle);
    if (angleVariation > M_PI)
        angleVariation -= 2 * M_PI;
    return angleVariation;
}

TEST(ParkingPathGenerator, ParallelParkingRight)
{
    ParallelParking parallelParking;
    int parkingPhase = 3;
    itri_msgs::CarState carState;
    carState.pose.pose.position.x = 18.0f;
    carState.pose.pose.position.y = 6.0f;
    ParkingPlannerUtils::ParkingPath output;

    itri_msgs::WaypointArray globalPathXY;
    float diffX = 0.1f * std::cos(M_PI / 6.0);
    float diffY = 0.1f * std::sin(M_PI / 6.0);

    for(int i = 0; i < 20.0 / 0.1; i++)
    {
        itri_msgs::Waypoint tmpPathXY;
        tmpPathXY.pose.pose.position.x = diffX * i;
        tmpPathXY.pose.pose.position.y = diffY * i;
        tmpPathXY.pose.pose.orientation.z = M_PI / 6.0;

        globalPathXY.waypoints.push_back(tmpPathXY);
    }

    itri_msgs::WaypointArray parkingSpaceXY;
    itri_msgs::Waypoint parkingCorner;

    parkingCorner.pose.pose.position.x = 10.34f;
    parkingCorner.pose.pose.position.y = -3.268f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 19.0f;
    parkingCorner.pose.pose.position.y = 1.732f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 12.38f;
    parkingCorner.pose.pose.position.y = -6.732f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 21.0f;
    parkingCorner.pose.pose.position.y = -1.732f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);

    ParkingPlannerUtils::ParkingSpaceInfo parkingSpaceInfo;
    ParkingPlannerUtils::ParkingSpaceInfoUpdate(parkingSpaceXY,
        globalPathXY, parkingSpaceInfo);

    parallelParking.ParameterUpdate(
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead, parkingSpaceXY);

    parallelParking.PathGenerator(parkingPhase,
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead,
        carState, parkingSpaceXY, output);

    itri_msgs::WaypointArray answer;
    LoadPathData("parking_path_generator/parallelpathanswer_case1.json", answer);
    for (size_t i = 0; i < answer.waypoints.size(); i ++)
    {
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.x,
            output.path.waypoints[i].pose.pose.position.x, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.y,
            output.path.waypoints[i].pose.pose.position.y, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.orientation.z,
            output.path.waypoints[i].pose.pose.orientation.z, SMALL_VALUE);
    }
}

TEST(ParkingPathGenerator, ParallelParkingLeft)
{
    ParallelParking parallelParking;
    int parkingPhase = 3;
    itri_msgs::CarState carState;
    carState.pose.pose.position.x = 18.0f;
    carState.pose.pose.position.y = 6.0f;
    ParkingPlannerUtils::ParkingPath output;

    itri_msgs::WaypointArray globalPathXY;
    float diffX = 0.1f * std::cos(M_PI / 6.0);
    float diffY = 0.1f * std::sin(M_PI / 6.0);

    for(int i = 0; i < 20.0 / 0.1; i++)
    {
        itri_msgs::Waypoint tmpPathXY;
        tmpPathXY.pose.pose.position.x = diffX * i;
        tmpPathXY.pose.pose.position.y = diffY * i;
        tmpPathXY.pose.pose.orientation.z = M_PI / 6.0;

        globalPathXY.waypoints.push_back(tmpPathXY);
    }

    itri_msgs::WaypointArray parkingSpaceXY;
    itri_msgs::Waypoint parkingCorner;

    carState.pose.pose.position.x = 16.0f;
    carState.pose.pose.position.y = 8.0f;

    parkingCorner.pose.pose.position.x = 5.3f;
    parkingCorner.pose.pose.position.y = 5.4f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 9.6f;
    parkingCorner.pose.pose.position.y = 7.9f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 3.7f;
    parkingCorner.pose.pose.position.y = 8.0f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 8.0f;
    parkingCorner.pose.pose.position.y = 10.5f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);

    ParkingPlannerUtils::ParkingSpaceInfo parkingSpaceInfo;
    ParkingPlannerUtils::ParkingSpaceInfoUpdate(parkingSpaceXY,
        globalPathXY, parkingSpaceInfo);

    parallelParking.ParameterUpdate(
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead, parkingSpaceXY);

    parallelParking.PathGenerator(parkingPhase,
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead,
        carState, parkingSpaceXY, output);

    itri_msgs::WaypointArray answer;
    LoadPathData("parking_path_generator/parallelpathanswer_case2.json", answer);
    for (size_t i = 0; i < answer.waypoints.size(); i ++)
    {
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.x,
            output.path.waypoints[i].pose.pose.position.x, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.y,
            output.path.waypoints[i].pose.pose.position.y, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.orientation.z,
            output.path.waypoints[i].pose.pose.orientation.z, SMALL_VALUE);
    }
}

TEST(ParkingPathGenerator, VerticalParkingRight)
{
    VerticalParking verticalParking;
    int parkingPhase = 3;
    itri_msgs::CarState carState;
    carState.pose.pose.position.x = 13.0f;
    carState.pose.pose.position.y = 8.0f;
    ParkingPlannerUtils::ParkingPath output;

    itri_msgs::WaypointArray globalPathXY;
    float diffX = 0.1f * std::cos(M_PI / 6.0);
    float diffY = 0.1f * std::sin(M_PI / 6.0);

    for(int i = 0; i < 20.0 / 0.1; i++)
    {
        itri_msgs::Waypoint tmpPathXY;
        tmpPathXY.pose.pose.position.x = diffX * i;
        tmpPathXY.pose.pose.position.y = diffY * i;
        tmpPathXY.pose.pose.orientation.z = M_PI / 6.0;

        globalPathXY.waypoints.push_back(tmpPathXY);
    }
    /*=============== case1 ================*/

    itri_msgs::WaypointArray parkingSpaceXY;
    itri_msgs::Waypoint parkingCorner;

    parkingCorner.pose.pose.position.x = 8.0f;
    parkingCorner.pose.pose.position.y = 0.0f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 10.2f;
    parkingCorner.pose.pose.position.y = 1.3f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 10.0f;
    parkingCorner.pose.pose.position.y = -3.4f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 12.2f;
    parkingCorner.pose.pose.position.y = -2.2f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);

    ParkingPlannerUtils::ParkingSpaceInfo parkingSpaceInfo;
    ParkingPlannerUtils::ParkingSpaceInfoUpdate(parkingSpaceXY,
        globalPathXY, parkingSpaceInfo);

    verticalParking.ParameterUpdate(
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead, parkingSpaceXY);

    verticalParking.PathGenerator(parkingPhase,
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead,
        carState, output);

    itri_msgs::WaypointArray answer;
    LoadPathData("parking_path_generator/verticalpathanswer_case1.json", answer);
    for (size_t i = 0; i < answer.waypoints.size(); i ++)
    {
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.x,
            output.path.waypoints[i].pose.pose.position.x, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.y,
            output.path.waypoints[i].pose.pose.position.y, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.orientation.z,
            output.path.waypoints[i].pose.pose.orientation.z, SMALL_VALUE);
    }
}

TEST(ParkingPathGenerator, VerticalParkingLeft)
{
    VerticalParking verticalParking;
    int parkingPhase = 3;
    itri_msgs::CarState carState;
    carState.pose.pose.position.x = 13.0f;
    carState.pose.pose.position.y = 8.0f;
    ParkingPlannerUtils::ParkingPath output;

    itri_msgs::WaypointArray globalPathXY;
    float diffX = 0.1f * std::cos(M_PI / 6.0);
    float diffY = 0.1f * std::sin(M_PI / 6.0);

    for(int i = 0; i < 20.0 / 0.1; i++)
    {
        itri_msgs::Waypoint tmpPathXY;
        tmpPathXY.pose.pose.position.x = diffX * i;
        tmpPathXY.pose.pose.position.y = diffY * i;
        tmpPathXY.pose.pose.orientation.z = M_PI / 6.0;

        globalPathXY.waypoints.push_back(tmpPathXY);
    }

    itri_msgs::WaypointArray parkingSpaceXY;
    itri_msgs::Waypoint parkingCorner;

    carState.pose.pose.position.x = 16.0f;
    carState.pose.pose.position.y = 5.0f;

    parkingCorner.pose.pose.position.x = 5.3f;
    parkingCorner.pose.pose.position.y = 5.4f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 7.4f;
    parkingCorner.pose.pose.position.y = 6.6f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 3.2f;
    parkingCorner.pose.pose.position.y = 8.8f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 5.5f;
    parkingCorner.pose.pose.position.y = 10.1f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);

    ParkingPlannerUtils::ParkingSpaceInfo parkingSpaceInfo;
    ParkingPlannerUtils::ParkingSpaceInfoUpdate(parkingSpaceXY,
        globalPathXY, parkingSpaceInfo);

    verticalParking.ParameterUpdate(
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead, parkingSpaceXY);

    verticalParking.PathGenerator(parkingPhase,
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead,
        carState, output);

    itri_msgs::WaypointArray answer;
    LoadPathData("parking_path_generator/verticalpathanswer_case2.json", answer);
    for (size_t i = 0; i < answer.waypoints.size(); i ++)
    {
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.x,
            output.path.waypoints[i].pose.pose.position.x, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.y,
            output.path.waypoints[i].pose.pose.position.y, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.orientation.z,
            output.path.waypoints[i].pose.pose.orientation.z, SMALL_VALUE);
    }
}


TEST(ParkingPathGenerator, ObliqueParkingRight)
{
    ObliqueParking obliqueParking;
    itri_msgs::plan behaivorMsg;
    int parkingPhase = 3;
    itri_msgs::CarState carState;
    carState.pose.pose.position.x = 14.0f;
    carState.pose.pose.position.y = 6.0f;
    ParkingPlannerUtils::ParkingPath output;

    itri_msgs::WaypointArray globalPathXY;
    float diffX = 0.1f * std::cos(M_PI / 6.0);
    float diffY = 0.1f * std::sin(M_PI / 6.0);

    for(int i = 0; i < 20.0 / 0.1; i++)
    {
        itri_msgs::Waypoint tmpPathXY;
        tmpPathXY.pose.pose.position.x = diffX * i;
        tmpPathXY.pose.pose.position.y = diffY * i;
        tmpPathXY.pose.pose.orientation.z = M_PI / 6.0;

        globalPathXY.waypoints.push_back(tmpPathXY);
    }
    itri_msgs::Path path;
    path.waypoints = globalPathXY.waypoints;
    GlobalPath globalPath(path);

    /*=============== case1 ================*/

    itri_msgs::WaypointArray parkingSpaceXY;
    itri_msgs::Waypoint parkingCorner;

    parkingCorner.pose.pose.position.x = 8.0f;
    parkingCorner.pose.pose.position.y = 0.0f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 10.2f;
    parkingCorner.pose.pose.position.y = 1.3f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 8.0f;
    parkingCorner.pose.pose.position.y = -5.0f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 10.2f;
    parkingCorner.pose.pose.position.y = -3.7f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);

    ParkingPlannerUtils::ParkingSpaceInfo parkingSpaceInfo;
    ParkingPlannerUtils::ParkingSpaceInfoUpdate(parkingSpaceXY,
        globalPathXY, parkingSpaceInfo);

    obliqueParking.ParameterUpdate(
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead, parkingSpaceXY);

    obliqueParking.PathGenerator(parkingPhase,
        parkingSpaceInfo.mParkingLocation, globalPath,
        parkingSpaceInfo.mSearchPathHead,
        behaivorMsg, carState, output);

    itri_msgs::WaypointArray answer;
    LoadPathData("parking_path_generator/obliquepathanswer_case1.json", answer);
    for (size_t i = 0; i < answer.waypoints.size(); i ++)
    {
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.x,
            output.path.waypoints[i].pose.pose.position.x, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.y,
            output.path.waypoints[i].pose.pose.position.y, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.orientation.z,
            output.path.waypoints[i].pose.pose.orientation.z, SMALL_VALUE);
    }
}

TEST(ParkingPathGenerator, ObliqueParkingLeft)
{
    ObliqueParking obliqueParking;
    itri_msgs::plan behaivorMsg;
    int parkingPhase = 3;
    itri_msgs::CarState carState;
    carState.pose.pose.position.x = 14.0f;
    carState.pose.pose.position.y = 6.0f;
    ParkingPlannerUtils::ParkingPath output;

    itri_msgs::WaypointArray globalPathXY;
    float diffX = 0.1f * std::cos(M_PI / 6.0);
    float diffY = 0.1f * std::sin(M_PI / 6.0);

    for(int i = 0; i < 20.0 / 0.1; i++)
    {
        itri_msgs::Waypoint tmpPathXY;
        tmpPathXY.pose.pose.position.x = diffX * i;
        tmpPathXY.pose.pose.position.y = diffY * i;
        tmpPathXY.pose.pose.orientation.z = M_PI / 6.0;

        globalPathXY.waypoints.push_back(tmpPathXY);
    }
    itri_msgs::Path path;
    path.waypoints = globalPathXY.waypoints;
    GlobalPath globalPath(path);

    itri_msgs::WaypointArray parkingSpaceXY;
    itri_msgs::Waypoint parkingCorner;

    carState.pose.pose.position.x = 17.0f;
    carState.pose.pose.position.y = 7.5f;

    parkingCorner.pose.pose.position.x = 7.4f;
    parkingCorner.pose.pose.position.y = 6.6f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 9.6f;
    parkingCorner.pose.pose.position.y = 7.9f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 3.2f;
    parkingCorner.pose.pose.position.y = 8.8f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 5.5f;
    parkingCorner.pose.pose.position.y = 10.1f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);

    ParkingPlannerUtils::ParkingSpaceInfo parkingSpaceInfo;
    ParkingPlannerUtils::ParkingSpaceInfoUpdate(parkingSpaceXY,
        globalPathXY, parkingSpaceInfo);

    obliqueParking.ParameterUpdate(
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead, parkingSpaceXY);

    obliqueParking.PathGenerator(parkingPhase,
        parkingSpaceInfo.mParkingLocation, globalPath,
        parkingSpaceInfo.mSearchPathHead,
        behaivorMsg, carState, output);

    itri_msgs::WaypointArray answer;
    LoadPathData("parking_path_generator/obliquepathanswer_case2.json", answer);
    for (size_t i = 0; i < answer.waypoints.size(); i ++)
    {
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.x,
            output.path.waypoints[i].pose.pose.position.x, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.y,
            output.path.waypoints[i].pose.pose.position.y, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.orientation.z,
            output.path.waypoints[i].pose.pose.orientation.z, SMALL_VALUE);
    }
}

TEST(ParkingPathGenerator, ObliqueLeaving_Left)
{
    ObliqueParking obliqueParking;
    itri_msgs::plan behaivorMsg;
    behaivorMsg.bias = 0.5;

    int parkingPhase = 9;
    itri_msgs::CarState carState;
    carState.pose.pose.position.x = 14.0f;
    carState.pose.pose.position.y = 6.0f;
    ParkingPlannerUtils::ParkingPath output;

    itri_msgs::WaypointArray globalPathXY;
    float diffX = 0.1f * std::cos(M_PI / 6.0);
    float diffY = 0.1f * std::sin(M_PI / 6.0);

    for(int i = 0; i < 1000.0 / 0.1; i++)
    {
        itri_msgs::Waypoint tmpPathXY;
        tmpPathXY.pose.pose.position.x = diffX * i;
        tmpPathXY.pose.pose.position.y = diffY * i;
        tmpPathXY.pose.pose.orientation.z = M_PI / 6.0;

        globalPathXY.waypoints.push_back(tmpPathXY);
    }
    itri_msgs::Path path;
    path.waypoints = globalPathXY.waypoints;
    GlobalPath globalPath(path);

    itri_msgs::WaypointArray parkingSpaceXY;
    itri_msgs::Waypoint parkingCorner;

    carState.pose.pose.position.x = 17.0f;
    carState.pose.pose.position.y = 7.5f;

    parkingCorner.pose.pose.position.x = 7.4f;
    parkingCorner.pose.pose.position.y = 6.6f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 9.6f;
    parkingCorner.pose.pose.position.y = 7.9f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 3.2f;
    parkingCorner.pose.pose.position.y = 8.8f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);
    parkingCorner.pose.pose.position.x = 5.5f;
    parkingCorner.pose.pose.position.y = 10.1f;
    parkingSpaceXY.waypoints.push_back(parkingCorner);

    ParkingPlannerUtils::ParkingSpaceInfo parkingSpaceInfo;
    ParkingPlannerUtils::ParkingSpaceInfoUpdate(parkingSpaceXY,
        globalPathXY, parkingSpaceInfo);

    obliqueParking.ParameterUpdate(
        parkingSpaceInfo.mParkingLocation,
        parkingSpaceInfo.mSearchPathHead, parkingSpaceXY);

    obliqueParking.PathGenerator(parkingPhase,
        parkingSpaceInfo.mParkingLocation, globalPath,
        parkingSpaceInfo.mSearchPathHead,
        behaivorMsg, carState, output);

    itri_msgs::WaypointArray answer;
    LoadPathData("parking_path_generator/obliquepathanswer_case3.json", answer);
    for (size_t i = 0; i < answer.waypoints.size(); i ++)
    {
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.x,
            output.path.waypoints[i].pose.pose.position.x, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.position.y,
            output.path.waypoints[i].pose.pose.position.y, SMALL_VALUE);
        ASSERT_NEAR(answer.waypoints[i].pose.pose.orientation.z,
            output.path.waypoints[i].pose.pose.orientation.z, SMALL_VALUE);
    }
}
