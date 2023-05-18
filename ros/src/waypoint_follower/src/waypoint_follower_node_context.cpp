#include <iostream>
#include <ros/ros.h>
#include <waypoint_follower_node_context.h>

static const double GEAR_RATIO_DEFAULT = 19.6;
static const int NODE_RATE = 100;
static const double PREDICT_AHEAD_TIME_DEFAULT = 1.0;
static const double STEER_OFFSET_DEFAULT = 0.0;
static const double WAYPOINT_RES_DEFAULT = 1.0;

WaypointFollowerNodeContext::WaypointFollowerNodeContext()
    : mNode()
    , mRate(100)
    , mCarState()
    , mVehicleState()
    , mWaypoints()
    , mSpeedCmd()
    , mControllerFileDir()
    , mParamGearRatio(GEAR_RATIO_DEFAULT)
    , mParamPredictAheadTime(PREDICT_AHEAD_TIME_DEFAULT)
    , mParamSteerOffset(STEER_OFFSET_DEFAULT)
    , mParamWayptRes(WAYPOINT_RES_DEFAULT)
    , mParkingStatus({0, ParkingPhase::INITIAL})
{
    mSubCarState = mNode.subscribe(
        "car_state", 1, &WaypointFollowerNodeContext::CallbackCarState, this);
    mSubWaypoints = mNode.subscribe(
        "waypoints", 1, &WaypointFollowerNodeContext::CallbackWaypoint, this);
    mSubSpeedCmd = mNode.subscribe(
        "speed_cmd", 1, &WaypointFollowerNodeContext::CallbackSpeedCmd, this);
    mSubVehicleState = mNode.subscribe(
        "/vehicle_state", 1, &WaypointFollowerNodeContext::CallbackVehicleState, this);
    mSubGlobalPath = mNode.subscribe("/global_path", 1,
        &WaypointFollowerNodeContext::CallbackGlobalPath, this);

    ros::NodeHandle privateNode("~");
    privateNode.param<double>(
        "gear_ratio", mParamGearRatio, GEAR_RATIO_DEFAULT);
    privateNode.param<double>(
        "predict_ahead_time", mParamPredictAheadTime, PREDICT_AHEAD_TIME_DEFAULT);
    privateNode.param<double>(
        "steering_offset", mParamSteerOffset, STEER_OFFSET_DEFAULT);
    privateNode.param<double>(
        "waypt_resolution", mParamWayptRes, WAYPOINT_RES_DEFAULT);

    const auto sdcConfigDir = std::getenv("SDC_CONFIG_DIR");
    if (sdcConfigDir)
        mControllerFileDir = sdcConfigDir;
    else
        ROS_ERROR("SDC_CONFIG_DIR environment variable wasn't set");

    mCarState.is_stable = false;
}

void WaypointFollowerNodeContext::SpinOnce()
{
    ros::spinOnce();
}

void WaypointFollowerNodeContext::CallbackCarState(
    const itri_msgs::CarState & msg)
{
    mCarState = msg;
}

void WaypointFollowerNodeContext::CallbackWaypoint(
    const itri_msgs::WaypointArray & msg)
{
    mWaypoints = msg;
}

void WaypointFollowerNodeContext::CallbackSpeedCmd(
    const itri_msgs::speed_cmd & msg)
{
    mSpeedCmd = msg.kph;
}

void WaypointFollowerNodeContext::CallbackVehicleState(
    const itri_msgs::VehicleState & msg)
{
    mVehicleState = msg;
}

void WaypointFollowerNodeContext::CallbackGlobalPath(
    const itri_msgs::Path & msg)
{
    mGlobalPath = std::make_shared<GlobalPath>(msg);
}
