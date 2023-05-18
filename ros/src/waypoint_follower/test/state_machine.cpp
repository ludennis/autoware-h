#include <file_loader.h>
#include <gtest/gtest.h>
#include <move_forward_state.h>
#include <move_reverse_state.h>
#include <state.h>
#include <waypoint_follower_node_context.h>
#include <ros/ros.h>
#include <string.h>

static const float SMALL_VALUE = 0.001f;

TEST(StateMachine, DoControl)
{
    std::vector<float> pathX;
    std::vector<float> pathY;
    LoadPathData("path/high_resolution_path.json", pathX, pathY);

    WaypointFollowerNodeContext context;

    context.mControllerFileDir = TEST_DATA_DIR;
    context.mControllerFileDir += "state_test/";

    std::shared_ptr<State> state = std::make_shared<MoveForwardState>(context);

    context.mWaypoints.waypoints.resize(pathX.size());
    for (size_t i = 0; i < context.mWaypoints.waypoints.size(); i ++)
    {
        context.mWaypoints.waypoints[i].pose.pose.position.x = pathX[i];
        context.mWaypoints.waypoints[i].pose.pose.position.y = pathY[i];
        if (i == context.mWaypoints.waypoints.size() - 1)
            context.mWaypoints.waypoints[i].pose.pose.orientation.z =
                context.mWaypoints.waypoints[i-1].pose.pose.orientation.z;
        else
            context.mWaypoints.waypoints[i].pose.pose.orientation.z =
                std::atan2(pathY[i + 1] - pathY[i], pathX[i + 1] - pathX[i]);
    }
    context.mParamWayptRes = 0.1f;
    context.mParamPredictAheadTime = 1.5f;
    context.mCarState.pose.pose.orientation.z =
        context.mWaypoints.waypoints.front().pose.pose.orientation.z;
    context.mCarState.twist.twist.linear.x = 2.0;
    context.mCarState.pose.pose.position.x = pathX.back();
    context.mCarState.pose.pose.position.y = pathY.back();

    float steeringCmd;
    context.mCarState.pose.pose.position.x = pathX.front() + 0.5f;
    context.mCarState.pose.pose.position.y = pathY.front();
    state = std::make_shared<MoveForwardState>(context);
    std::vector<float> answer;
    LoadAnswerFromFile("state_test/answer.json", answer);
    for (auto const & value : answer)
    {
        state = state->Handle(context, steeringCmd);
        ASSERT_NEAR(steeringCmd, value, SMALL_VALUE);
    }
}

TEST(StateMachine, StateChange)
{
    std::string dir = TEST_DATA_DIR;
    dir += "state_test/";

    ros::param::set("/vehicle_configuration_directory", dir);

    WaypointFollowerNodeContext context;
    context.mWaypoints.waypoints.resize(10);

    for (auto & waypoint : context.mWaypoints.waypoints)
    {
        waypoint.twist.twist.linear.x = -1.0;
    }
    context.mSpeedCmd.twist.linear.x = -1.0;

    std::shared_ptr<State> state = std::make_shared<MoveForwardState>(context);
    ASSERT_EQ(state->GetClassID(), MoveForwardState::ID());

    float steeringCmd;
    state = state->Handle(context, steeringCmd);
    ASSERT_EQ(state->GetClassID(), MoveReverseState::ID());

    for (auto & waypoint : context.mWaypoints.waypoints)
    {
        waypoint.twist.twist.linear.x = 0.0;
    }
    context.mSpeedCmd.twist.linear.x = 0.0;
    state = state->Handle(context, steeringCmd);
    ASSERT_EQ(state->GetClassID(), MoveForwardState::ID());
}
