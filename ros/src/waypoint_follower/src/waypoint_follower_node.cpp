#include <itri_msgs/steer_cmd.h>
#include <iostream>
#include <move_forward_state.h>
#include <moving_range_matching.h>
#include <signal_filter.h>
#include <state.h>
#include <waypoint_follower.h>
#include <waypoint_follower_node.h>

static const float INITIAL_VALUE_ZERO = 0.0f;
static const float SPEED_FILTER_FACTOR = 100.0f;
static const float YAW_RATE_FILTER_FACTOR = 100.0f;
static int indexMatch = 0;

static uint8_t oldVehicleMode = itri_msgs::VehicleState::MANUAL;

static bool ReEngagement(
    const WaypointFollowerNodeContext & node, const float steeringCmd)
{
    const bool modeReEngage = (
        node.mVehicleState.mode - oldVehicleMode > 0);
    if (std::abs(node.mVehicleState.steering_angle - steeringCmd) < 10.0f ||
        node.mVehicleState.mode == itri_msgs::VehicleState::MANUAL)
        oldVehicleMode = node.mVehicleState.mode;
    return modeReEngage;
}

WaypointFollowerNode::WaypointFollowerNode()
    : mNode()
    , mPubControlCmd()
    , mContext()
{
    mPubControlCmd =
        mNode.advertise<itri_msgs::steer_cmd>("/steer_cmd", 1);
}

void WaypointFollowerNode::MainLoop()
{
    MovingRangeMatching pathMatch;
    float steeringCmd = INITIAL_VALUE_ZERO;

    SignalFilter speedFilter(SPEED_FILTER_FACTOR);
    SignalFilter yawRateFilter(YAW_RATE_FILTER_FACTOR);

    std::shared_ptr<State> state = std::make_shared<MoveForwardState>(mContext);

    while (ros::ok())
    {
        mContext.SpinOnce();

        if (mContext.mGlobalPath)
        {
            indexMatch = pathMatch.RunOnce(
                mContext.mGlobalPath->mResolution, mContext.mGlobalPath->mPath,
                mContext.mCarState);
            const int globalPathSize =
                mContext.mGlobalPath->mPath.waypoints.size();

            if (mContext.mCarState.is_stable && HaveWaypoints())
            {
                mContext.mCarState.twist.twist.linear.x =
                    speedFilter.OneStep(mContext.mCarState.twist.twist.linear.x);
                mContext.mCarState.twist.twist.angular.z =
                    yawRateFilter.OneStep(mContext.mCarState.twist.twist.angular.z);
                if (indexMatch >= globalPathSize - 7)
                {
                    steeringCmd -= 0.04f * steeringCmd;
                }
                else
                {
                    state = state->Handle(mContext, steeringCmd);
                }
            }

            float steerCmdTmp = steeringCmd;
            float offset = 70.0f - mContext.mCarState.twist.twist.linear.x * 5.0f;
            offset = std::max(offset, 10.0f);
            float reEngageGain = 1.0f - 0.06f * mContext.mCarState.twist.twist.linear.x;
            reEngageGain = std::max(reEngageGain, 0.2f);
            if (ReEngagement(mContext, steeringCmd))
                steerCmdTmp = mContext.mVehicleState.steering_angle +
                    reEngageGain * (steeringCmd - mContext.mVehicleState.steering_angle);

            itri_msgs::steer_cmd steerCmd;
            steerCmd.type = itri_msgs::steer_cmd::CLOSE_LOOP;
            steerCmd.angle = steerCmdTmp;

            mPubControlCmd.publish(steerCmd);
        }
        mContext.mRate.sleep();
    }
}

bool WaypointFollowerNode::HaveWaypoints()
{
    return mContext.mWaypoints.waypoints.size() > 1;
}
