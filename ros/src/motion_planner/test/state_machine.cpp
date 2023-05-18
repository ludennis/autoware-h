#include <file_loader.h>
#include <gtest/gtest.h>
#include <motion_planner/along_global_path_state.h>
#include <motion_planner/custom_path_state.h>
#include <motion_planner/idle_state.h>
#include <motion_planner/moving_range_matching.h>
#include <motion_planner/state.h>

static inline float PrincipleAngle(const float angle)
{
    float anglePrcp = std::atan2(std::sin(angle), std::cos(angle));
    if (std::signbit(anglePrcp))
        anglePrcp += 2.0f * M_PI;
    return anglePrcp;
}

static const float SMALL_VALUE = 1.0e-02f;

TEST(StateMachine, AlongGlobalPathState)
{
    MovingRangeMatching movingRangeMatching;
    MotionPlannerContext context;
    ros::NodeHandle nodeHandle;

    itri_msgs::Path path;
    itri_msgs::WaypointArray pathTmp;
    LoadPathData("state_machine/curve_path.json", pathTmp);
    path.waypoints.assign(pathTmp.waypoints.begin(), pathTmp.waypoints.end());
    context.mGlobalPath = std::make_shared<GlobalPath>(path);

    context.mCarState.pose = context.mGlobalPath->mPath.waypoints[1].pose;
    context.mCarState.twist.twist.linear.x = 0.0;
    context.mBehavior.num_level = 2;
    context.mBehavior.bias = 0.0f;
    context.mBehavior.dev = 0.4f;
    context.mPlannerParam.rollOutDist = 20.0f;
    context.mPlannerParam.rollInFactor = 1.0f;
    context.mPlannerParam.wayptRes = 0.1f;

    context.mPubWaypoints = nodeHandle.advertise<itri_msgs::WaypointArray>(
        "/waypoints", 0);
    context.mPubLocalPaths = nodeHandle.advertise<itri_msgs::WaypointArrays>(
        "/local_paths", 0);
    context.mPubParkingPathStatus =
        nodeHandle.advertise<itri_msgs::ParkingStatus>(
            "/parking_path_status", 0);

    const int indexMatch = movingRangeMatching.RunOnce(
        context.mGlobalPath->mResolution, context.mGlobalPath->mPath,
        context.mCarState);

    std::shared_ptr<State> localPathState =
        std::make_shared<AlongGlobalPathState>();
    localPathState = localPathState->Generate(indexMatch, context);

    itri_msgs::WaypointArrays pathAnswer;
    LoadPathData("state_machine/local_path1.json", pathTmp);
    pathAnswer.waypointArrays.push_back(pathTmp);
    LoadPathData("state_machine/local_path2.json", pathTmp);
    pathAnswer.waypointArrays.push_back(pathTmp);
    LoadPathData("state_machine/local_path3.json", pathTmp);
    pathAnswer.waypointArrays.push_back(pathTmp);

    ASSERT_EQ(pathAnswer.waypointArrays.size(),
        context.mLocalPaths.waypointArrays.size());
    for (size_t i = 0; i < context.mLocalPaths.waypointArrays.size(); ++ i)
    {
        ASSERT_EQ(pathAnswer.waypointArrays[i].waypoints.size(),
            context.mLocalPaths.waypointArrays[i].waypoints.size());
        for (size_t j = 0;
            j < context.mLocalPaths.waypointArrays[i].waypoints.size(); ++ j)
        {
            ASSERT_NEAR(
                pathAnswer.waypointArrays[i].waypoints[j].pose.pose.position.x,
                context.mLocalPaths.waypointArrays[i].waypoints[j].pose.pose.position.x,
                SMALL_VALUE);
            ASSERT_NEAR(
                pathAnswer.waypointArrays[i].waypoints[j].pose.pose.position.y,
                context.mLocalPaths.waypointArrays[i].waypoints[j].pose.pose.position.y,
                SMALL_VALUE);
            ASSERT_NEAR(
                PrincipleAngle(
                    pathAnswer.waypointArrays[i].waypoints[j].pose.pose.orientation.z),
                context.mLocalPaths.waypointArrays[i].waypoints[j].pose.pose.orientation.z,
                SMALL_VALUE);
        }
    }
}

TEST(StateMachine, CustomPathStateParallelParking)
{
    MovingRangeMatching movingRangeMatching;
    MotionPlannerContext context;
    ros::NodeHandle nodeHandle;

    context.mPubWaypoints = nodeHandle.advertise<itri_msgs::WaypointArray>(
        "/waypoints", 0);
    context.mPubLocalPaths = nodeHandle.advertise<itri_msgs::WaypointArrays>(
        "/local_paths", 0);
    context.mPubParkingPathStatus =
        nodeHandle.advertise<itri_msgs::ParkingStatus>(
            "/parking_path_status", 0);

    itri_msgs::Path path;
    itri_msgs::WaypointArray pathTmp;
    LoadPathData("state_machine/straight_path.json", pathTmp);
    path.waypoints.assign(pathTmp.waypoints.begin(), pathTmp.waypoints.end());
    context.mGlobalPath = std::make_shared<GlobalPath>(path);

    context.mCarState.pose.pose.position.x = 18.0f;
    context.mCarState.pose.pose.position.y = 6.0f;
    context.mCarState.pose.pose.orientation.z = M_PI / 6.0f;

    context.mCarState.twist.twist.linear.x = 0.0f;
    context.mBehavior.num_level = 0;
    context.mPlannerParam.wayptRes = 0.1f;

    itri_msgs::Waypoint parkingCorner;

    parkingCorner.pose.pose.position.x = 10.34f;
    parkingCorner.pose.pose.position.y = -3.268f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 19.0f;
    parkingCorner.pose.pose.position.y = 1.732f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 12.38f;
    parkingCorner.pose.pose.position.y = -6.732f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 21.0f;
    parkingCorner.pose.pose.position.y = -1.732f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);

    context.mParkingStatus.parking_space.type = 1;
    context.mParkingStatus.phase = 3;

    const int indexMatch = movingRangeMatching.RunOnce(
        context.mGlobalPath->mResolution, context.mGlobalPath->mPath,
        context.mCarState);

    std::shared_ptr<State> localPathState =
        std::make_shared<CustomPathState>();
    localPathState = localPathState->Generate(indexMatch, context);

    itri_msgs::WaypointArrays pathAnswer;
    LoadPathData("state_machine/parallelpathanswer_case1.json", pathTmp);
    pathAnswer.waypointArrays.push_back(pathTmp);

    ASSERT_EQ(pathAnswer.waypointArrays[0].waypoints.size(),
        context.mLocalPaths.waypointArrays[0].waypoints.size());
    for (size_t j = 0;
        j < context.mLocalPaths.waypointArrays[0].waypoints.size(); ++ j)
    {
        ASSERT_NEAR(
            pathAnswer.waypointArrays[0].waypoints[j].pose.pose.position.x,
            context.mLocalPaths.waypointArrays[0].waypoints[j].pose.pose.position.x,
            SMALL_VALUE);
        ASSERT_NEAR(
            pathAnswer.waypointArrays[0].waypoints[j].pose.pose.position.y,
            context.mLocalPaths.waypointArrays[0].waypoints[j].pose.pose.position.y,
            SMALL_VALUE);
        ASSERT_NEAR(
            PrincipleAngle(
                pathAnswer.waypointArrays[0].waypoints[j].pose.pose.orientation.z),
            context.mLocalPaths.waypointArrays[0].waypoints[j].pose.pose.orientation.z,
            SMALL_VALUE);
    }
}

TEST(StateMachine, CustomPathStateVerticalParking)
{
    MovingRangeMatching movingRangeMatching;
    MotionPlannerContext context;
    ros::NodeHandle nodeHandle;

    context.mPubWaypoints = nodeHandle.advertise<itri_msgs::WaypointArray>(
        "/waypoints", 0);
    context.mPubLocalPaths = nodeHandle.advertise<itri_msgs::WaypointArrays>(
        "/local_paths", 0);
    context.mPubParkingPathStatus =
        nodeHandle.advertise<itri_msgs::ParkingStatus>(
            "/parking_path_status", 0);

    itri_msgs::Path path;
    itri_msgs::WaypointArray pathTmp;
    LoadPathData("state_machine/straight_path.json", pathTmp);
    path.waypoints.assign(pathTmp.waypoints.begin(), pathTmp.waypoints.end());
    context.mGlobalPath = std::make_shared<GlobalPath>(path);

    context.mCarState.pose.pose.position.x = 13.0f;
    context.mCarState.pose.pose.position.y = 8.0f;
    context.mCarState.pose.pose.orientation.z = M_PI / 6.0f;

    context.mCarState.twist.twist.linear.x = 0.0f;
    context.mBehavior.num_level = 0;
    context.mPlannerParam.wayptRes = 0.1f;

    itri_msgs::Waypoint parkingCorner;

    parkingCorner.pose.pose.position.x = 8.0f;
    parkingCorner.pose.pose.position.y = 0.0f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 10.2f;
    parkingCorner.pose.pose.position.y = 1.3f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 10.0f;
    parkingCorner.pose.pose.position.y = -3.4f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 12.2f;
    parkingCorner.pose.pose.position.y = -2.2f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);

    context.mParkingStatus.parking_space.type = 0;
    context.mParkingStatus.phase = 3;

    const int indexMatch = movingRangeMatching.RunOnce(
        context.mGlobalPath->mResolution, context.mGlobalPath->mPath,
        context.mCarState);

    std::shared_ptr<State> localPathState =
        std::make_shared<CustomPathState>();
    localPathState = localPathState->Generate(indexMatch, context);

    itri_msgs::WaypointArrays pathAnswer;
    LoadPathData("state_machine/verticalpathanswer_case1.json", pathTmp);
    pathAnswer.waypointArrays.push_back(pathTmp);

    ASSERT_EQ(pathAnswer.waypointArrays[0].waypoints.size(),
        context.mLocalPaths.waypointArrays[0].waypoints.size());
    for (size_t j = 0;
        j < context.mLocalPaths.waypointArrays[0].waypoints.size(); ++ j)
    {
        ASSERT_NEAR(
            pathAnswer.waypointArrays[0].waypoints[j].pose.pose.position.x,
            context.mLocalPaths.waypointArrays[0].waypoints[j].pose.pose.position.x,
            SMALL_VALUE);
        ASSERT_NEAR(
            pathAnswer.waypointArrays[0].waypoints[j].pose.pose.position.y,
            context.mLocalPaths.waypointArrays[0].waypoints[j].pose.pose.position.y,
            SMALL_VALUE);
        ASSERT_NEAR(
            PrincipleAngle(
                pathAnswer.waypointArrays[0].waypoints[j].pose.pose.orientation.z),
            context.mLocalPaths.waypointArrays[0].waypoints[j].pose.pose.orientation.z,
            SMALL_VALUE);
    }
}

TEST(StateMachine, CustomPathStateObliqueParking)
{
    MovingRangeMatching movingRangeMatching;
    MotionPlannerContext context;
    ros::NodeHandle nodeHandle;

    context.mPubWaypoints = nodeHandle.advertise<itri_msgs::WaypointArray>(
        "/waypoints", 0);
    context.mPubLocalPaths = nodeHandle.advertise<itri_msgs::WaypointArrays>(
        "/local_paths", 0);
    context.mPubParkingPathStatus =
        nodeHandle.advertise<itri_msgs::ParkingStatus>(
            "/parking_path_status", 0);

    itri_msgs::Path path;
    itri_msgs::WaypointArray pathTmp;
    LoadPathData("state_machine/straight_path.json", pathTmp);
    path.waypoints.assign(pathTmp.waypoints.begin(), pathTmp.waypoints.end());
    context.mGlobalPath = std::make_shared<GlobalPath>(path);

    context.mCarState.pose.pose.position.x = 14.0f;
    context.mCarState.pose.pose.position.y = 6.0f;
    context.mCarState.pose.pose.orientation.z = M_PI / 6.0f;

    context.mCarState.twist.twist.linear.x = 0.0f;
    context.mBehavior.num_level = 0;
    context.mPlannerParam.wayptRes = 0.1f;

    itri_msgs::Waypoint parkingCorner;

    parkingCorner.pose.pose.position.x = 8.0f;
    parkingCorner.pose.pose.position.y = 0.0f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 10.2f;
    parkingCorner.pose.pose.position.y = 1.3f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 8.0f;
    parkingCorner.pose.pose.position.y = -5.0f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 10.2f;
    parkingCorner.pose.pose.position.y = -3.7f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);

    context.mParkingStatus.parking_space.type = 3;
    context.mParkingStatus.phase = 3;

    const int indexMatch = movingRangeMatching.RunOnce(
        context.mGlobalPath->mResolution, context.mGlobalPath->mPath,
        context.mCarState);

    std::shared_ptr<State> localPathState =
        std::make_shared<CustomPathState>();
    localPathState = localPathState->Generate(indexMatch, context);

    itri_msgs::WaypointArrays pathAnswer;
    LoadPathData("state_machine/obliquepathanswer_case1.json", pathTmp);
    pathAnswer.waypointArrays.push_back(pathTmp);

    ASSERT_EQ(pathAnswer.waypointArrays[0].waypoints.size(),
        context.mLocalPaths.waypointArrays[0].waypoints.size());
    for (size_t j = 0;
        j < context.mLocalPaths.waypointArrays[0].waypoints.size(); ++ j)
    {
        ASSERT_NEAR(
            pathAnswer.waypointArrays[0].waypoints[j].pose.pose.position.x,
            context.mLocalPaths.waypointArrays[0].waypoints[j].pose.pose.position.x,
            SMALL_VALUE);
        ASSERT_NEAR(
            pathAnswer.waypointArrays[0].waypoints[j].pose.pose.position.y,
            context.mLocalPaths.waypointArrays[0].waypoints[j].pose.pose.position.y,
            SMALL_VALUE);
        ASSERT_NEAR(
            PrincipleAngle(
                pathAnswer.waypointArrays[0].waypoints[j].pose.pose.orientation.z),
            context.mLocalPaths.waypointArrays[0].waypoints[j].pose.pose.orientation.z,
            SMALL_VALUE);
    }
}

TEST(StateMachine, StateTransition)
{
    std::shared_ptr<State> localPathState =
        std::make_shared<IdleState>();
    MotionPlannerContext context;
    ros::NodeHandle nodeHandle;

    context.mPubWaypoints = nodeHandle.advertise<itri_msgs::WaypointArray>(
        "/waypoints", 0);
    context.mPubLocalPaths = nodeHandle.advertise<itri_msgs::WaypointArrays>(
        "/local_paths", 0);
    context.mPubParkingPathStatus =
        nodeHandle.advertise<itri_msgs::ParkingStatus>(
            "/parking_path_status", 0);

    itri_msgs::Waypoint parkingCorner;
    parkingCorner.pose.pose.position.x = 8.0f;
    parkingCorner.pose.pose.position.y = 0.0f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 10.2f;
    parkingCorner.pose.pose.position.y = 1.3f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 10.0f;
    parkingCorner.pose.pose.position.y = -3.4f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    parkingCorner.pose.pose.position.x = 12.2f;
    parkingCorner.pose.pose.position.y = -2.2f;
    context.mParkingStatus.parking_space.array.waypoints.push_back(
        parkingCorner);
    context.mParkingStatus.parking_space.type = 0;

    itri_msgs::Path path;
    itri_msgs::WaypointArray pathTmp;
    LoadPathData("state_machine/straight_path.json", pathTmp);
    path.waypoints.assign(pathTmp.waypoints.begin(), pathTmp.waypoints.end());
    context.mGlobalPath = std::make_shared<GlobalPath>(path);

    context.mBehavior.hint = true;
    context.mBehavior.parking_start = true;
    context.mParkingStatus.phase = 0;
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), IdleState::ID());

    context.mParkingStatus.phase = 2;
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), IdleState::ID());

    context.mParkingStatus.phase = 5;
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), IdleState::ID());

    context.mParkingStatus.phase = 6;
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), IdleState::ID());

    context.mParkingStatus.phase = 7;
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), IdleState::ID());

    context.mParkingStatus.phase = 8;
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), IdleState::ID());

    context.mParkingStatus.phase = 1;
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), AlongGlobalPathState::ID());

    context.mLocalPaths.waypointArrays.push_back(pathTmp);
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), IdleState::ID());

    context.mBehavior.hint = true;
    context.mParkingStatus.phase = 3;
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), CustomPathState::ID());

    context.mLocalPaths.waypointArrays.push_back(pathTmp);
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), IdleState::ID());

    context.mBehavior.hint = true;
    context.mParkingStatus.phase = 4;
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), CustomPathState::ID());

    context.mLocalPaths.waypointArrays.push_back(pathTmp);
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), IdleState::ID());

    context.mBehavior.hint = true;
    context.mBehavior.parking_start = false;
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), AlongGlobalPathState::ID());

    context.mLocalPaths.waypointArrays.push_back(pathTmp);
    localPathState = localPathState->Generate(-1, context);
    ASSERT_EQ(localPathState->GetClassID(), IdleState::ID());
}
