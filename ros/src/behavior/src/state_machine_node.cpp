#define DEBUG_TREE

#include <behavior/state_machine_node.h>
#include <behavior/lane_following.h>
#include <behavior/aeb.h>
#include <behavior/obstacle_avoidance.h>
#include <behavior/get_curvature.h>
#include <stdint.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <behavior/avoidance_sequence.h>
#include <behavior/avoidance_fallback.h>
#include <behavior/behavior_parallel.h>
#include <behavior/lane_change/lane_change_sequence.h>
#include <trace/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsoncpp/json/json.h>
#include <fstream>

static Json::Reader JSON_READER;

#ifdef DEBUG_TREE
#include <dot_bt.h>
#include <std_msgs/String.h>
#endif // DEBUG_TREE

static const float CURVATURE_MAX = 125.0f;

// #define _PK_DEBUG_
// #define _OA_DEBUG_
// #define _LF_DEBUG_
// #define _IA_DEBUG_
// #define _FSM_DEBUG_

static visualization_msgs::Marker InitLineMarker(
    const std::vector<double> & rgb)
{
    visualization_msgs::Marker marker;
    marker.ns = "points_and_lines";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.2;
    marker.color.r = rgb[0];
    marker.color.g = rgb[1];
    marker.color.b = rgb[2];
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.05);
    return marker;
}

static visualization_msgs::Marker GetPathMarker()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.ns = "points_and_lines";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    return marker;
}

using hash_t = size_t;
constexpr hash_t prime = 0x100000001B3ull;
constexpr hash_t basis = 0xCBF29CE484222325ull;

static hash_t hash_run_time(const char* str) {
    hash_t ret = basis;

    while (*str) {
        ret ^= *str;
        ret *= prime;
        str++;
    }

    return ret;
}

constexpr hash_t hash_compile_time(const char* str, hash_t last_value = basis) {
    return *str ? hash_compile_time(str + 1, (*str ^ last_value) * prime) : last_value;
}

constexpr hash_t operator "" _hash(const char* p, size_t) {
    return hash_compile_time(p);
}

static void AngularTransform(float & x, float & y, const float theta)
{
    const float tmpX = x;
    const float tmpY = y;
    x = tmpX * std::cos(theta) + tmpY * std::sin(theta);
    y = -tmpX * std::sin(theta) + tmpY * std::cos(theta);
}

namespace Behavior
{
StateMachineNode::StateMachineNode()
    : mTargetAcc(0.0)
    , mSpeed(0.0)
    , mTargetSpeed(0.0)
    , mTime(ros::Time::now().toSec())
    , mNodeSpeedCommand(std::make_shared<SpeedCmd>())
    , mContext(std::make_shared<Context>())
    , mCurve(false)
    , mTrafficStopLineCount(0)
    , mBumperCount(0)
    , mTrafficLight(2)
    , mTrafficLightFake(2)
    , mTrafficLightSensor(2)
    , mTimeStep(0.0)
    , mTreeTimeStep(0.0)
    , mAccRoiWaypoints()
    , mPubGlobalPathMarker()
{
    mContext->pubBehaviorPlan = mNode.advertise<itri_msgs::plan>("/behavior", 1);
    mPubBehaviorState = mNode.advertise<itri_msgs::BehaviorState>("/behavior_state", 1);
    mPubGlobalPath = mNode.advertise<itri_msgs::Path>("/global_path", 1, true);
    mPubGlobalPathMarker = mNode.advertise<visualization_msgs::Marker>("/global_path/marker", 1, true);
    mPubTurnCmd = mNode.advertise<itri_msgs::turn_signal_cmd>("/turn_signal_cmd",1);
    mPubSpeedCmd =
        mNode.advertise<itri_msgs::speed_cmd>("/speed_cmd", 1);
    mSubVehPos = mNode.subscribe("/car_state", 1,
        &StateMachineNode::CallbackVehPos, this);
    mSubLocalWayPoints = mNode.subscribe("/waypoints", 1,
        &StateMachineNode::CallbackWayPoint, this);
    mSubDetectionBoundary = mNode.subscribe("/detection_boundary", 0,
        &StateMachineNode::CallbackDetectionBoundary, this);
    mSubRadarAssoci = mNode.subscribe("/radar_objs_associ", 1,
        &StateMachineNode::CallbackRadarAssociObj, this);
    mSubObjList = mNode.subscribe("/detected_objects", 1,
        &StateMachineNode::CallbackObList,this);
    mSubLaneInfo = mNode.subscribe("/lane_info", 1,
        &StateMachineNode::CallbackLaneType, this);
    mSubRouteInfo = mNode.subscribe("/route_info", 1,
        &StateMachineNode::CallbackRouteInfo, this);
    mSubLaneWaypointsInfo = mNode.subscribe("lane_waypoints_info", 1,
        &StateMachineNode::CallbackLaneWaypointsInfo, this);
    mSubIntersection = mNode.subscribe("/intersection",1,
        &StateMachineNode::CallbackIntersection, this);
    mSubVehicleOdometry = mNode.subscribe("/vehicle_state",1,
        &StateMachineNode::CallbackVehicleOdometry, this);
    mSubTrafficLightStateSensor = mNode.subscribe("/traffic_light_status",1,
        &StateMachineNode::CallbackTrafficLightObjectsSensor, this);
    mSubTrafficLightStateFake = mNode.subscribe("/traffic_light_status_fake",1,
        &StateMachineNode::CallbackTrafficLightObjectsFake, this);
    mSubNokiaAlarm = mNode.subscribe("/nokia_alarm",1,
        &StateMachineNode::CallbackNokiaAlarm, this);
    mSubPlatooning = mNode.subscribe("/platooning_car_state",1,
        &StateMachineNode::CallbackPlatooning, this);
    mSubNaviPath = mNode.subscribe("/navigation_path",1,
        &StateMachineNode::CallbackNavigationPath, this);

    mContext->pubActionMarkers = mNode.advertise<visualization_msgs::MarkerArray>(
        "viz_Action_Markers", 1);
    ros::NodeHandle nodeHandle("~");
    nodeHandle.param<double>("car_max_speed", mMaxSpeed, 0.0);
    nodeHandle.param<double>("waypt_resolution", mWayptRes, 1.0);
    nodeHandle.param<double>("pid_p_gain", mLaneFollowing.mPGain, 0.001);
    nodeHandle.param<double>("pid_i_gain", mLaneFollowing.mIGain, 0.001);
    nodeHandle.param<double>("pid_d_gain", mLaneFollowing.mDGain, 0.0);
    nodeHandle.param<double>("roll_in_factor", mContext->paramRollInFactor, 5.0);
    nodeHandle.getParam("vehicle", mVehicleName);
    LoadIntersectionROIFromFile("labels_intersections_crosswalk_operation_new.json");
    LoadIntersectionFOVROIFromFile("labels_intersections_operation_new.json");

    mNode.getParam("route", mRoute);

    mContext->debug = std::make_shared<DebugToFile>(
        std::to_string(ros::Time::now().toSec()) +
        "debug_data.json");
    mContext->debug->SetLogOptionOff();
    mContext->avoidanceNeedDistance = false;
}

template <typename T>
static inline T Clamp(const T & value, T bottom, T top)
{
    return std::max(bottom, std::min(top, value));
}

void StateMachineNode::LoadIntersectionROIFromFile(
    const std::string & fileName)
{
    std::vector<std::vector<int>> intersectionROI;
    std::string fullFileName = DATA_DIR + fileName;
    std::ifstream file(fullFileName);
    JSON_READER.parse(file, mContext->intersectionROIs);
}

void StateMachineNode::LoadIntersectionFOVROIFromFile(
    const std::string & fileName)
{
    std::vector<std::vector<int>> intersectionFOVROI;
    std::string fullFileName = DATA_DIR + fileName;
    std::ifstream file(fullFileName);
    JSON_READER.parse(file, mContext->intersectionFOVROIs);
}

bool StateMachineNode::CheckTransformListener(
    tf::StampedTransform & transform, const ros::Time & stamp)
{
    bool succesfulLookup = false;
    try
    {
        mTransformListener.lookupTransform(
            "/map", "/base_link", stamp, transform);
        succesfulLookup = true;
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        ROS_WARN("Look up transform Failure!!!");
        succesfulLookup = false;
    }
}

void StateMachineNode::CallbackTrafficLightObjectsSensor(
    const itri_msgs::TrafficLightObjects& msg)
{
    if (msg.lights.size() > 0)
    {
        if (!msg.lights[0].red &&
            !msg.lights[0].green &&
            !msg.lights[0].yellow &&
            !msg.lights[0].straight &&
            !msg.lights[0].left &&
            !msg.lights[0].right)
        {
            mTrafficLightSensor = 2;
            return;
        }
        else if (msg.lights[0].red &&
            !msg.lights[0].green &&
            !msg.lights[0].yellow &&
            !msg.lights[0].straight)
        {
            mTrafficLightSensor = 1;
            if (msg.lights[0].left &&
                !msg.lights[0].right)
                mTrafficLightSensor = 5;

            else if (!msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightSensor = 6;

            else if (msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightSensor = 7;

        }
        else if (!msg.lights[0].red &&
            msg.lights[0].green &&
            !msg.lights[0].yellow &&
            !msg.lights[0].straight)
            mTrafficLightSensor = 2;

        else if (!msg.lights[0].red &&
            !msg.lights[0].green &&
            msg.lights[0].yellow &&
            !msg.lights[0].straight)
            mTrafficLightSensor = 3;

        else if (!msg.lights[0].red &&
            !msg.lights[0].green &&
            !msg.lights[0].yellow &&
            msg.lights[0].straight)
        {
            mTrafficLightSensor = 4;
            if (msg.lights[0].left &&
                !msg.lights[0].right)
                mTrafficLightSensor = 8;

            else if (!msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightSensor = 9;

            else if (msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightSensor = 10;
        }
        else
        {
            if (msg.lights[0].left &&
                !msg.lights[0].right)
                mTrafficLightSensor = 11;

            else if (!msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightSensor = 12;

            else if (msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightSensor = 13;
        }
    }
    else
        mTrafficLightSensor = 2;
}

void StateMachineNode::CallbackTrafficLightObjectsFake(
    const itri_msgs::TrafficLightObjects& msg)
{
    if (msg.lights.size() > 0)
    {
        if (!msg.lights[0].red &&
            !msg.lights[0].green &&
            !msg.lights[0].yellow &&
            !msg.lights[0].straight &&
            !msg.lights[0].left &&
            !msg.lights[0].right)
        {
            mTrafficLightFake = 2;
            mContext->trafficLightFake = mTrafficLightFake;
            return;
        }
        else if (msg.lights[0].red &&
            !msg.lights[0].green &&
            !msg.lights[0].yellow &&
            !msg.lights[0].straight)
        {
            mTrafficLightFake = 1;
            if (msg.lights[0].left &&
                !msg.lights[0].right)
                mTrafficLightFake = 5;

            else if (!msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightFake = 6;

            else if (msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightFake = 7;

        }
        else if (!msg.lights[0].red &&
            msg.lights[0].green &&
            !msg.lights[0].yellow &&
            !msg.lights[0].straight)
            mTrafficLightFake = 2;

        else if (!msg.lights[0].red &&
            !msg.lights[0].green &&
            msg.lights[0].yellow &&
            !msg.lights[0].straight)
            mTrafficLightFake = 3;

        else if (!msg.lights[0].red &&
            !msg.lights[0].green &&
            !msg.lights[0].yellow &&
            msg.lights[0].straight)
        {
            mTrafficLightFake = 4;
            if (msg.lights[0].left &&
                !msg.lights[0].right)
                mTrafficLightFake = 8;

            else if (!msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightFake = 9;

            else if (msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightFake = 10;
        }
        else
        {
            if (msg.lights[0].left &&
                !msg.lights[0].right)
                mTrafficLightFake = 11;

            else if (!msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightFake = 12;

            else if (msg.lights[0].left &&
                msg.lights[0].right)
                mTrafficLightFake = 13;
        }

        mContext->trafficLightFake = mTrafficLightFake;
    }
    else
    {
        mTrafficLightFake = 2;
        mContext->trafficLightFake = mTrafficLightFake;
    }
}

void StateMachineNode::TrafficLightStatusDecision()
{
    if (mTrafficLightFake == 2)
    {
        mTrafficLight = mTrafficLightSensor;
        mContext->trafficLight = mTrafficLightSensor;
    }
    else
    {
        mTrafficLight = mTrafficLightFake;
        mContext->trafficLight = mTrafficLightFake;
    }
}

void StateMachineNode::CallbackPlatooning(
    const itri_msgs::CarStateConstPtr& msg)
{
    mFleetVehPosXY.x = static_cast<float>(msg->pose.pose.position.x);
    mFleetVehPosXY.y = static_cast<float>(msg->pose.pose.position.y);
    mFleetVehPosXY.heading = static_cast<float>(msg->pose.pose.orientation.z);
    mFleetVehPosXY.speed = static_cast<float>(msg->twist.twist.linear.x);
    mFleetVehPosXY.yawRate = static_cast<float>(msg->twist.twist.angular.z);
    mFleetVehPosXY.stable = msg->is_stable;

    mContext->fleetVehPosXY = mFleetVehPosXY;
    mContext->CACC = true;
}

void StateMachineNode::CallbackVehPos(
    const itri_msgs::CarStateConstPtr& msg)
{
    mVehPosXY.x = static_cast<float>(msg->pose.pose.position.x);
    mVehPosXY.y = static_cast<float>(msg->pose.pose.position.y);
    mVehPosXY.heading = static_cast<float>(msg->pose.pose.orientation.z);
    mVehPosXY.speed = static_cast<float>(msg->twist.twist.linear.x);
    mVehPosXY.yawRate = static_cast<float>(msg->twist.twist.angular.z);
    mVehPosXY.stable = msg->is_stable;

    std::lock_guard<std::mutex> guard(mContext->mutex);

    mContext->relativeWayPointsX.clear();
    mContext->relativeWayPointsY.clear();
    mContext->relativeWayPointsHeading.clear();
    mContext->mWayPointsHead.clear();

    mContext->vehPosXY.x = static_cast<float>(msg->pose.pose.position.x);
    mContext->vehPosXY.y = static_cast<float>(msg->pose.pose.position.y);
    mContext->vehPosXY.heading = static_cast<float>(msg->pose.pose.orientation.z);
    mContext->vehPosXY.speed = static_cast<float>(msg->twist.twist.linear.x);
    mContext->vehPosXY.yawRate = static_cast<float>(msg->twist.twist.angular.z);
    mContext->vehPosXY.stable = msg->is_stable;

    for (int i = 0; i < mWayPointsX.size(); i++)
    {
        const float localRelativeX =
            std::cos(mContext->vehPosXY.heading)*(mWayPointsX[i] - mContext->vehPosXY.x) +
            std::sin(mContext->vehPosXY.heading)*(mWayPointsY[i] - mContext->vehPosXY.y);
        const float localRelativeY =
            -std::sin(mContext->vehPosXY.heading)*(mWayPointsX[i] - mContext->vehPosXY.x) +
            std::cos(mContext->vehPosXY.heading)*(mWayPointsY[i] - mContext->vehPosXY.y);
        mContext->relativeWayPointsX.push_back(localRelativeX);
        mContext->relativeWayPointsY.push_back(localRelativeY);
        mContext->relativeWayPointsHeading.push_back(mWayPointsHead[i] - mContext->vehPosXY.heading);
        mContext->mWayPointsHead.push_back(mWayPointsHead[i]);
    }

    mContext->relativeOriginX =
        std::cos(mContext->vehPosXY.heading)*(- mContext->vehPosXY.x) +
        std::sin(mContext->vehPosXY.heading)*(- mContext->vehPosXY.y);
    mContext->relativeOriginY =
        -std::sin(mContext->vehPosXY.heading)*(0 - mContext->vehPosXY.x) +
        std::cos(mContext->vehPosXY.heading)*(0 - mContext->vehPosXY.y);
}

void StateMachineNode::CallbackWayPoint(
    const itri_msgs::WaypointArrayConstPtr& msg)
{
    mWayPointsX.clear();
    mWayPointsY.clear();
    mContext->wayPointsX.clear();
    mContext->wayPointsY.clear();
    mWayPointsHead.clear();
    mWayPointsSpeed.clear();

    mWayPointArry = *msg;
    for (int i = 0; i < msg->waypoints.size(); i++)
    {
        mWayPointsX.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.position.x));
        mWayPointsY.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.position.y));
        mContext->wayPointsX.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.position.x));
        mContext->wayPointsY.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.position.y));
        mWayPointsHead.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.orientation.z));
        mWayPointsSpeed.push_back(
            static_cast<float>(msg->waypoints[i].twist.twist.linear.x));
    }
}

void StateMachineNode::CallbackNavigationPath(
    const route_mission_handler::Path::ConstPtr& msg)
{
    std::map<int, std::map<int ,route_mission_handler::Waypoint>> emptyLane;
    std::map<int, route_mission_handler::Lane> emptyLaneInfo;
    mContext->laneMap.swap(emptyLane); //release memory
    mContext->laneInfoMap.swap(emptyLaneInfo);
    mContext->laneMap.clear(); //clear map
    mContext->laneInfoMap.clear();
    mContext->naviLane.clear();
    mContext->naviPath = *msg;

    for (const auto & lane: msg->lanes)
    {
        mContext->naviLane.push_back(lane);
        mContext->laneInfoMap.insert(std::pair<int,
            route_mission_handler::Lane>(lane.lane_id, lane));
        std::map<int ,route_mission_handler::Waypoint> waypointsMap;
        for(const auto & waypoint: lane.waypoints)
        {
            waypointsMap.insert(std::pair<int,
                route_mission_handler::Waypoint>(waypoint.point_id, waypoint));
        }
        mContext->laneMap.insert(std::pair<int,
            std::map<int, route_mission_handler::Waypoint>>(lane.lane_id, waypointsMap));
    }
}

void StateMachineNode::CalculateGlobalPathProperty()
{
    mContext->globalPathX.clear();
    mContext->globalPathY.clear();
    mContext->globalPathZ.clear();
    mContext->globalPathS.clear();
    mContext->globalPathD.resize(mContext->globalPath.size(), 0.0f);
    mContext->globalPathHead.clear();
    mContext->globalPathCurvature.clear();
    mBumper.clear();
    mTrafficStopLine.clear();

    for (int i = 0; i < mContext->globalPath.size(); i++)
    {
        for (int j = 0; j < mContext->globalPath[i].markerTypes.size(); j++)
        {
            if (mContext->globalPath[i].markerTypes[j] == 14)
                mTrafficStopLine.push_back(static_cast<int>(i));
            if (mContext->globalPath[i].markerTypes[j] == 29)
                mBumper.push_back(static_cast<int>(i));
        }
    }

    float s = 0.0f;
    itri_msgs::Path globalPathToPub;
    itri_msgs::Waypoint itriPt;
    auto pathMarker = GetPathMarker();
    for (const auto & waypoint : mContext->globalPath)
    {
        geometry_msgs::Point markerPt;
        markerPt.x = waypoint.point.x;
        markerPt.y = waypoint.point.y;
        markerPt.z = waypoint.point.z;
        pathMarker.points.push_back(markerPt);
        float heading = 0.0f;
        if (!!mContext->globalPathX.size())
            heading = std::atan2(
                waypoint.point.y - mContext->globalPathY.back(),
                waypoint.point.x - mContext->globalPathX.back());

        mContext->globalPathX.push_back(waypoint.point.x);
        mContext->globalPathY.push_back(waypoint.point.y);
        mContext->globalPathZ.push_back(waypoint.point.z);
        mContext->globalPathHead.push_back(heading);
        mContext->globalPathS.push_back(s ++);

        itriPt.pose.pose.position.x = waypoint.point.x;
        itriPt.pose.pose.position.y = waypoint.point.y;
        itriPt.pose.pose.position.z = waypoint.point.z;
        itriPt.pose.pose.orientation.w = waypoint.curvature;
        globalPathToPub.waypoints.push_back(itriPt);
    }

    const auto tmp = mContext->globalPathHead.back();
    std::rotate(mContext->globalPathHead.begin(),
        mContext->globalPathHead.begin() + 1, mContext->globalPathHead.end());
    mContext->globalPathHead.back() = tmp;

    GetCurvature(mContext->globalPathX, mContext->globalPathY,
        mContext->globalPathCurvature);

    for (size_t i{ 0u }; i < globalPathToPub.waypoints.size(); ++ i)
    {
        globalPathToPub.waypoints[i].pose.pose.orientation.z =
            mContext->globalPathHead[i];
        globalPathToPub.waypoints[i].pose.pose.orientation.w =
            mContext->globalPathCurvature[i];
        mContext->globalPathCurvature[i] = 1.0f /
            std::abs(static_cast<float>(mContext->globalPathCurvature[i]));
        mContext->globalPathCurvature[i] = std::min(CURVATURE_MAX,
            mContext->globalPathCurvature[i]);
    }
    mPubGlobalPath.publish(globalPathToPub);
    mLaneFollowing.CurveGetStartEnd(
        mContext->globalPathCurvature, mIndexStartCurve, mIndexEndCurve);

    mPubGlobalPathMarker.publish(pathMarker);
}

void StateMachineNode::CallbackDetectionBoundary(
    const geometry_msgs::PolygonStamped & msg)
{
    mContext->detectionBoundary = msg.polygon;
}

void StateMachineNode::CallbackRadarAssociObj(
    const itri_msgs::DetectedObjectArrayConstPtr& msg)
{
    Core behaviorCore;

    std::lock_guard<std::mutex> guard(mContext->mutex);

    mContext->objRadarXY.clear();

    bool succesfulLookup = CheckTransformListener(mTransform, msg->header.stamp);

    if (msg->objects.size() > 0)
    {
        for (const auto & object: msg->objects)
        {
            ObjectXY objPoints;
            behaviorCore.TransformObjectXY(
                mTransform,
                objPoints,
                object);
            mContext->objRadarXY.push_back(objPoints);
        }
    }
}

void StateMachineNode::CallbackObList(
    const itri_msgs::DetectedObjectArrayConstPtr& msg)
{
    Core behaviorCore;

    std::lock_guard<std::mutex> guard(mContext->mutex);

    mContext->mObjBaseLink = *msg;

    mContext->objListXY.clear();
    mContext->objListX.clear();
    mContext->objListY.clear();
    mObjWidth.clear();

    bool succesfulLookup = CheckTransformListener(mTransform, msg->header.stamp);
#ifdef _FSM_DEBUG_
    mContext->debug->AddDebug("Objects size", msg->objects.size());
#endif
    if (msg->objects.size() > 0)
    {
        for (const auto & object: msg->objects)
        {
            if (object.id < 400 && object.convex_hull.polygon.points.size() > 0)
            {
                ObjectXY objPoints;
                mContext->objListX.push_back(object.pose.position.x);
                mContext->objListY.push_back(object.pose.position.y);
                behaviorCore.TransformObjectXY(
                    mTransform,
                    objPoints,
                    object);

                float diffSpeedX = objPoints.speedX;
                float diffSpeedY = objPoints.speedY;
                AngularTransform(
                    diffSpeedX, diffSpeedY, mContext->vehPosXY.heading);
                diffSpeedX -= mContext->vehPosXY.speed;

                float diffX = objPoints.centerX - mContext->vehPosXY.x;
                float diffY = objPoints.centerY - mContext->vehPosXY.y;
                AngularTransform(
                    diffX, diffY, mContext->vehPosXY.heading + M_PI_2);

                objPoints.relativeSpeedX =
                    diffSpeedX + diffX * mContext->vehPosXY.yawRate;
                objPoints.relativeSpeedY =
                    diffSpeedY + diffY * mContext->vehPosXY.yawRate;

                mContext->objListXY.push_back(objPoints);
            }

        }
        for (int i = 0; i < msg->objects.size(); ++i)
            mObjWidth.push_back(0);
    }
}

void StateMachineNode::CallbackIntersection(
    const itri_msgs::WaypointArrayConstPtr& msg)
{
    mGlobalCrossX.clear();
    mGlobalCrossY.clear();
    mGlobalCrossHead.clear();
    mGlobalCrossS.clear();
    mGlobalCrossD.clear();

    for (int i = 0; i < msg->waypoints.size(); i++)
    {
        mGlobalCrossX.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.position.x));
        mGlobalCrossY.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.position.y));
        mGlobalCrossHead.push_back(
            static_cast<float>(msg->waypoints[i].pose.pose.orientation.z));
    }
}

void StateMachineNode::CallbackLaneType(
    const itri_msgs::lane_infoConstPtr& msg)
{
    mBumper.clear();
    std::lock_guard<std::mutex> guard(mContext->mutex);

    mContext->laneId = msg->lane_id;
    mContext->laneNum = msg->lane_num;
    mContext->laneType.clear();
    mOnRouteHint.clear();
    for (int i = 0; i < msg->line_type.size(); i++)
        mContext->laneType.push_back(static_cast<LaneType>(msg->line_type[i]));

    for (int i = 0; i < msg->on_route_hint.size(); i++)
        mOnRouteHint.push_back(static_cast<int>(msg->on_route_hint[i]));

    if(msg->start_right_turn.size() == msg->end_right_turn.size())
    {
        mRightTurn.clear();
        for (int i = 0; i < msg->start_right_turn.size(); i++)
        {
            std::pair<int, int> range;
            range.first = msg->start_right_turn[i];
            range.second = msg->end_right_turn[i];
            mRightTurn.push_back(range);
        }
    }

    if(msg->start_left_turn.size() == msg->end_left_turn.size())
    {
        mLeftTurn.clear();
        for (int i = 0; i < msg->start_left_turn.size(); i++)
        {
            std::pair<int, int> range;
            range.first = msg->start_left_turn[i];
            range.second = msg->end_left_turn[i];
            mLeftTurn.push_back(range);
        }
    }
}


void StateMachineNode::CallbackRouteInfo(
    const itri_msgs::Route_infoConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(mContext->mutex);

    mContext->roadWidth.clear();
    for (int i = 0; i < msg->road_w.size(); i++)
        mContext->roadWidth.push_back(static_cast<float>(msg->road_w[i]));
}

void StateMachineNode::CallbackLaneWaypointsInfo(
    const itri_msgs::lane_waypoints_infoConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(mContext->mutex);

    mContext->laneData.clear();

    for (int i = 0; i < msg->point.size(); i++)
    {
        LaneInfo laneInfo;
        laneInfo.laneWidth = msg->point[i].lane_width;
        laneInfo.laneID = msg->point[i].lane_id;
        laneInfo.laneNum = msg->point[i].lane_num;
        laneInfo.leftType = static_cast<LaneType>(msg->point[i].line_type[0]);
        laneInfo.rightType = static_cast<LaneType>(msg->point[i].line_type[1]);
        mContext->laneData.push_back(laneInfo);
    }
}

void StateMachineNode::CallbackVehicleOdometry(
    const itri_msgs::VehicleStateConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(mContext->mutex);
    float gearRatio = 19.6f;
    mContext->gearState = msg->gear_state;
    mVehicleOdometry.speed = (msg->speed) / 3.6f;
    mVehicleOdometry.auto_mode = static_cast<bool>(msg->mode);
    mVehicleOdometry.steering =
        - (M_PI / 180.0f) * static_cast<float>(msg->steering_angle) / gearRatio;
    mContext->vehicleOdometry = mVehicleOdometry;
}

void StateMachineNode::CallbackNokiaAlarm(
    const std_msgs::Bool & msg)
{
    if (msg.data)
    {
        SpeedCmd cmd;
        cmd.speed = 8.0f / 3.6f;
        cmd.acceleration = -1.0f;
        mContext->accelerationCmd["NOKIA_ALARM"] = cmd;
    }
}

bool StateMachineNode::IsReady()
{
    return
        mWayPointsX.size() > 0 &&
        mContext->globalPathX.size() > 0 &&
        mContext->globalPathX.size() == mContext->globalPathCurvature.size() &&
        mContext->laneNum != 0 &&
        mVehPosXY.stable == true;
}

void StateMachineNode::SetMaxSpeed(std::string pathName, std::string vehicleName,
    float & maxSpeed, const int lastClosestPathPoint)
{
    if (pathName != "hsr")
    {
        if (vehicleName == "pacifica")
            maxSpeed = 30.0f / 3.6f;

        else if (vehicleName == "cpev")
            maxSpeed = 12.0f / 3.6f;
    }
    else
    {
        if (vehicleName == "pacifica")
        {
            maxSpeed = 55.0f / 3.6f;
            if (lastClosestPathPoint < 277 || lastClosestPathPoint > 15368)
               maxSpeed = 40.0f / 3.6f;

            else if ((lastClosestPathPoint < 1962 && lastClosestPathPoint > 1701) ||
               (lastClosestPathPoint < 6177 && lastClosestPathPoint > 5898) ||
               (lastClosestPathPoint < 13663 && lastClosestPathPoint > 13163) ||
               (lastClosestPathPoint < 8149 && lastClosestPathPoint > 6194) ||
               (lastClosestPathPoint < 8570 && lastClosestPathPoint > 8447))
               maxSpeed = 40.0f / 3.6f;

            else if ((lastClosestPathPoint < 12901 && lastClosestPathPoint > 12001) ||
               (lastClosestPathPoint < 5798 && lastClosestPathPoint > 4801))
               maxSpeed = 60.0f / 3.6f;

            else if ((lastClosestPathPoint > 8148 && lastClosestPathPoint < 8448) ||
                (lastClosestPathPoint > 690 && lastClosestPathPoint < 775))
               maxSpeed = 20.0f / 3.6f;
        }
        else if (vehicleName == "cpev")
        {
            maxSpeed = 40.0f / 3.6f;
            if (lastClosestPathPoint < 277 || lastClosestPathPoint > 15370)
                maxSpeed = 20.0f / 3.6f;

            else if ((lastClosestPathPoint < 1958 && lastClosestPathPoint > 1698) ||
                (lastClosestPathPoint < 6178 && lastClosestPathPoint > 5898))
                maxSpeed = 30.0f / 3.6f;

            else if ((lastClosestPathPoint < 12900 && lastClosestPathPoint > 12000) ||
                (lastClosestPathPoint < 5798 && lastClosestPathPoint > 4798))
                maxSpeed = 50.0f / 3.6f;

            else if (lastClosestPathPoint > 8149 && lastClosestPathPoint < 8449)
                maxSpeed = 20.0f / 3.6f;
        }
    }
    maxSpeed = 12.0f / 3.6f;
}

void StateMachineNode::UpdateTurnSignal(
    const int lastClosestPathPoint,
    const std::vector<std::pair<int, int>> & rightTurn,
    const std::vector<std::pair<int, int>> & lefttTurn,
    itri_msgs::turn_signal_cmd & turnCmd)
{
    turnCmd.turn_signal = mContext->turnSignalCmd.turn_signal;
    bool isUpdated = false;

    for(int i = 0; i < lefttTurn.size() && isUpdated == false; i++)
    {
        if (lefttTurn[i].first != 0 &&
            lefttTurn[i].second != 0 &&
            (lastClosestPathPoint - lefttTurn[i].first) >= 0 &&
            (lastClosestPathPoint - lefttTurn[i].second) <= 0)
        {
            turnCmd.turn_signal = itri_msgs::turn_signal_cmd::LEFT;
            isUpdated = true;
        }
    }

    for(int i = 0; i < rightTurn.size() && isUpdated == false; i++)
    {
        if (rightTurn[i].first != 0 &&
            rightTurn[i].second != 0 &&
            (lastClosestPathPoint - rightTurn[i].first) >= 0 &&
            (lastClosestPathPoint - rightTurn[i].second) <= 0)
        {
            turnCmd.turn_signal = itri_msgs::turn_signal_cmd::RIGHT;
            isUpdated = true;
        }
    }
}

void StateMachineNode::UpdateStateMachine()
{
    Core behaviorCore;
    BehaviorOutput behaviorOutput;
    SpeedCmd speedCmd;
    int objID = -1;

    std::lock_guard<std::mutex> guard(mContext->mutex);

    if (mContext->searchingRange == 0)
    {
        mContext->lastClosestPathPoint = 0;
        mContext->searchingRange = mContext->globalPathX.size();
    }

    mContext->wayPointsS.clear();
    mContext->wayPointsD.clear();
    mGlobalCrossS.clear();
    mGlobalCrossD.clear();

    behaviorCore.FrenetTransform(
        mVehPosXY.x,
        mVehPosXY.y,
        mVehPosXY.heading,
        mContext->globalPathX,
        mContext->globalPathY,
        mContext->vehPosSD.s,
        mContext->vehPosSD.d,
        mContext->lastClosestPathPoint,
        mContext->searchingRange,
        mContext->globalPathKdtree);
    mContext->vehPosSD.heading = mVehPosXY.heading;
    mContext->vehPosSD.speed = mVehPosXY.speed;
    mContext->vehPosSD.yawRate = mVehPosXY.yawRate;

    behaviorCore.FrenetTransform(
        mWayPointsX,
        mWayPointsY,
        mWayPointsHead,
        mContext->globalPathX,
        mContext->globalPathY,
        mContext->wayPointsS,
        mContext->wayPointsD,
        mContext->lastClosestPathPoint,
        mWayPointsX.size(),
        mContext->globalPathKdtree);

    //Obj frenet
    mContext->objListSD.clear();
    mContext->objRadarSD.clear();
    mContext->objLiRarXY.clear();
    mContext->objLiRarXYACC.clear();
    mContext->objLiRarSD.clear();
    //mContext->objListXY.clear();

    if (mContext->objListXY.size() != 0)
        behaviorCore.FrenetTransform(
            mContext->objListXY,
            mContext->globalPathX,
            mContext->globalPathY,
            mContext->globalPathHead,
            mVehPosXY,
            mContext->objListSD,
            mContext->lastClosestPathPoint,
            100.0,
            mVehicleOdometry.steering,
            mContext->globalPathKdtree);

    if (mContext->objRadarXY.size() != 0)
        behaviorCore.FrenetTransform(
            mContext->objRadarXY,
            mContext->globalPathX,
            mContext->globalPathY,
            mContext->globalPathHead,
            mVehPosXY,
            mContext->objRadarSD,
            mContext->lastClosestPathPoint,
            100.0,
            mVehicleOdometry.steering,
            mContext->globalPathKdtree);


    if (!mContext->objListXY.empty())
    {
        for (const auto & object: mContext->objListXY)
        {
            mContext->objLiRarXYACC.push_back(object);
            mContext->objLiRarXY[object.id] = object;
        }
        for (const auto & object: mContext->objListSD)
            mContext->objLiRarSD.push_back(object);
    }

    if (!mContext->objRadarXY.empty())
    {
        for (const auto & object: mContext->objRadarXY)
        {
            mContext->objLiRarXYACC.push_back(object);
            mContext->objLiRarXY[object.id] = object;
        }
        for (const auto & object: mContext->objRadarSD)
            mContext->objLiRarSD.push_back(object);
    }

    behaviorCore.FrenetTransform(
        mGlobalCrossX,
        mGlobalCrossY,
        mGlobalCrossHead,
        mContext->globalPathX,
        mContext->globalPathY,
        mGlobalCrossS,
        mGlobalCrossD,
        0,
        mContext->globalPathX.size(),
        mContext->globalPathKdtree);

    int vehLocalPosIndex = 0;
    float minLocalPosDist = 1e9;
    for (int i = 0; i < mWayPointsX.size(); ++i)
    {
        float distance =
            std::hypot((mWayPointsX[i] - mVehPosXY.x),
            (mWayPointsY[i] - mVehPosXY.y));
        if (minLocalPosDist > distance)
        {
            minLocalPosDist = distance;
            vehLocalPosIndex = i;
        }
    }

    if (mContext->lastClosestPathPoint < 10)
    {
        mBumperCount = 0;
        mTrafficStopLineCount = 0;
    }

    mNextTrafficStopLine = mTrafficStopLine[mTrafficStopLineCount];
    if ((mContext->lastClosestPathPoint > mNextTrafficStopLine) &&
        (mTrafficStopLineCount < mTrafficStopLine.size()))
        mTrafficStopLineCount++;

    mNextBumper = mBumper[mBumperCount];
    if (((mContext->lastClosestPathPoint - 5) > mNextBumper) &&
        (mBumperCount < mBumper.size()))
        mBumperCount++;

    behaviorCore.LocalPathROI(
        mWayPointArry,
        mWayPointsX,
        mWayPointsY,
        mVehPosXY,
        mAccRoiWaypoints);
    Polygon accRoiPoly = behaviorCore.LaneToPoly(mAccRoiWaypoints, 0.85f);

    float maxSpeed = 20.0f / 3.6f;
    SetMaxSpeed(mRoute, mVehicleName, maxSpeed, mContext->lastClosestPathPoint);
    const float wayPtRes = static_cast<float>(mWayptRes);
    const float pidPGain = static_cast<float>(mLaneFollowing.mPGain);
    const float pidIGain = static_cast<float>(mLaneFollowing.mIGain);
    const float pidDGain = static_cast<float>(mLaneFollowing.mDGain);
    const float rollInFactor = static_cast<float>(mContext->paramRollInFactor);
    const float twistCmdSpeed = mSpeed;

#ifdef _FSM_DEBUG_
    mContext->debug->AddDebug("MaxSpeed", maxSpeed * 3.6f);
#endif
    mContext->maxSpeed = maxSpeed;

    behaviorCore.BumperDetect(mContext->bumper, mContext->currentBumper,
        mNextBumper, mContext->lastClosestPathPoint, maxSpeed, mContext->vehPosSD.s);
    behaviorCore.CurveDetect(mContext->enterCurve, mContext->vehPosSD.s,
        mContext->globalPathS, mContext->globalPathCurvature, mIndexStartCurve, mIndexEndCurve,
        maxSpeed, mContext->curveSpeed);
    if (!mContext->enterCurve) mContext->curveSpeed = 0.0f;

    mLaneFollowing.UpdateTrafficStopLine(
        mContext->currentStopLine, mContext->vehPosSD.s,
        mNextTrafficStopLine, mTrafficLight);

    if(mContext->objLiRarSD.size() != 0)
    {
        float frontVelocity;
        bool obstacleHazard = false;
        mDeltaDistance = 0.0f;
        mShortestDistance = 0.0f;

        try
        {
            objID = mLaneFollowing.ClosestObjOnLocalPath(
                maxSpeed,
                mContext->objLiRarXYACC,
                mContext->objLiRarSD,
                accRoiPoly,
                mContext->vehPosSD,
                obstacleHazard,
                mShortestDistance,
                frontVelocity,
                mContext->actionMarkers["ACC"],
                mContext->ACCFrontObjs,
                mContext->ACCObj);
        }
        catch (...)
        {
            ROS_WARN("ClosestObjOnLocalPath dead !!!");
        }

        mContext->obstacleHazard = obstacleHazard;
        if (obstacleHazard)
        {
            mLaneFollowing.VelocitySetPoint(twistCmdSpeed, mContext->curveSpeed,
                mContext->bumper, mContext->enterCurve, maxSpeed, frontVelocity);
            mLaneFollowing.ACCOutput(
                mSpeed, twistCmdSpeed, maxSpeed, mContext->curveSpeed,
                mContext->bumper, mContext->enterCurve, mShortestDistance,
                frontVelocity, mTimeStep, pidPGain, pidIGain, pidDGain,
                mContext->accSpeed, mContext->accAccelerate, mDeltaDistance);
        }
        else
            mLaneFollowing.PIDRelease();
    }
    else
    {
        if (mContext->obstacleHazard)
        {
            mContext->obstacleHazard = false;
            mLaneFollowing.PIDRelease();
        }
    }

    mContext->aebCommand = mAEB.StateFunction(
        mBehavior,
        mContext->gearState,
        mContext->laneType,
        mVehPosXY,
        mContext->vehPosSD.s,
        mContext->vehPosSD.d,
        mContext->objListXY,
        mContext->objListSD,
        mGlobalCrossS,
        behaviorOutput,
        mBehaviorStateOut,
        mContext->mObjBaseLink,
        mContext->accelerationCmd,
        mContext->globalPathZ[mContext->lastClosestPathPoint]);

    if (mContext->aebCommand.state_hint == true)
        mBehavior = State::AEB;
    else
    {
        TrafficLightStatusDecision();
        mContext->actionMarkers["ACC"] = visualization_msgs::MarkerArray();

        mContext->ACCFrontObjs.clear();
        mContext->ACCObj.id = -1;
    }
}

void StateMachineNode::PubBehavior(
    const std::shared_ptr<BehaviorParallel> & behaviorTree)
{
    Core behaviorCore;
    BehaviorOutput behaviorOutput;
    bool avoidanceIsRunning = false;

    avoidanceIsRunning =
        behaviorTree->get_status() == BT::RUNNING && mNodeSpeedCommand->state_hint;
    behaviorCore.SetBehaviorOutput(
        mBehaviorPlanOut, behaviorOutput);

    mLastVehicleSpeed = mVehPosXY.speed;

    if (mBehaviorPlanOut.hint && !avoidanceIsRunning)
        mContext->pubBehaviorPlan.publish(mBehaviorPlanOut);

    const double timeNow = ros::Time::now().toSec();
    mTimeStep = timeNow - mTime;
    mContext->timeStep = mTimeStep;
    mTime = timeNow;
#ifdef _FSM_DEBUG_
    mContext->debug->AddDebug("xxx TimeStep", mTimeStep);
#endif
    itri_msgs::speed_cmd controlCmd;

    if (mVehicleOdometry.auto_mode)
    {
        behaviorCore.SetSpeedProfile(
            mContext->accelerationCmd, mSpeed, mTargetAcc,
            mTargetSpeed, mTimeStep, mContext->tagState, mBehavior);

        behaviorCore.DecisionState(mContext->tagState, mBehaviorStateOut,
            mTrafficLight, mContext->OAmMaxBiasId, mContext->OAmSlowDownObjId,
            mContext->IAobj.id, mContext->ACCObj.id, mContext->OAGenBias);
        mContext->OAGenBias = false;

        mContext->accelerationCmd.clear();
    }
    else
        mSpeed = mVehicleOdometry.speed;

    mContext->speed = mSpeed;

    controlCmd.type = itri_msgs::speed_cmd::CLOSE_LOOP;
    controlCmd.kph = mSpeed * 3.6f;

    mPubSpeedCmd.publish(controlCmd);
    mPubBehaviorState.publish(mBehaviorStateOut);

    //Update turn signal
    itri_msgs::turn_signal_cmd turnCmd;
    UpdateTurnSignal(mContext->lastClosestPathPoint,
        mRightTurn, mLeftTurn, turnCmd);
    mPubTurnCmd.publish(turnCmd);

    mContext->lastClosestPathPoint = behaviorCore.ClosestWaypoint(
        mVehPosXY.x,
        mVehPosXY.y,
        mContext->globalPathX,
        mContext->globalPathY,
        mContext->lastClosestPathPoint,
        mContext->searchingRange,
        mContext->globalPathKdtree);
    mContext->searchingRange = static_cast<int>(mVehPosXY.speed) + 10;

    //PubRvizLFObj(objID);

}

void StateMachineNode::Visualization()
{
    visualization_msgs::MarkerArray markerList;

    int markerId = 500;
    for (const auto & ACCFrontObj : mContext->ACCFrontObjs)
    {
        visualization_msgs::Marker lineMarker = InitLineMarker(
            {1.0, 1.0, 1.0});
        lineMarker.header.frame_id = "map";
        lineMarker.id = markerId ++;

        if (ACCFrontObj.x.size())
        {
            geometry_msgs::Point markerPt;
            for (size_t i = 0; i < ACCFrontObj.x.size(); ++ i)
            {
                markerPt.x = ACCFrontObj.x[i];
                markerPt.y = ACCFrontObj.y[i];
                markerPt.z = ACCFrontObj.z[i];
                lineMarker.points.push_back(markerPt);
            }
            markerPt.x = ACCFrontObj.x.front();
            markerPt.y = ACCFrontObj.y.front();
            markerPt.z = ACCFrontObj.z.front();
            lineMarker.points.push_back(markerPt);
            markerList.markers.push_back(lineMarker);
        }
    }

    if (hash_run_time(mContext->tagState.c_str()) == "ACC"_hash)
    {
        visualization_msgs::Marker lineMarker = InitLineMarker(
            {1.0, 0.0, 0.0});
        lineMarker.header.frame_id = "map";
        lineMarker.id = markerId ++;
        if (mContext->ACCObj.id != -1)
        {
            geometry_msgs::Point markerPt;
            for (size_t i = 0; i < mContext->ACCObj.x.size(); ++ i)
            {
                markerPt.x = mContext->ACCObj.x[i];
                markerPt.y = mContext->ACCObj.y[i];
                markerPt.z = mContext->ACCObj.z[i] + 0.3;
                lineMarker.points.push_back(markerPt);
            }
            markerPt.x = mContext->ACCObj.x.front();
            markerPt.y = mContext->ACCObj.y.front();
            markerPt.z = mContext->ACCObj.z.front() + 0.3;
            lineMarker.points.push_back(markerPt);
            markerList.markers.push_back(lineMarker);
        }
    }

    mContext->pubActionMarkers.publish(markerList);
}

void StateMachineNode::MainLoop()
{
    ros::Rate loop_rate(100);
    mBehavior = State::INITIAL;
    mContext->searchingRange = 0;

    const auto behaviorTree =
        std::make_shared<BehaviorParallel>("behaviorTree", mNodeSpeedCommand, mContext);

    const auto globalPathTree =
        std::make_shared<LaneChangeSequence>("globalPathTree", mContext);

    #ifdef DEBUG_TREE
    auto behaviorTreeGraphPublisher = mNode.advertise<std_msgs::String>(
        "tree_graph", 1);
    BT::DotBt behaviorTreeGraphBuilder(behaviorTree);
    #endif // DEBUG_TREE


    int counter = 0;
    while(ros::ok())
    {
        mContext->debug->StartAddDebug(ros::Time::now().toSec());
        ros::spinOnce();
        if (globalPathTree->Tick() == BT::SUCCESS)
            CalculateGlobalPathProperty();

        if (IsReady())
        {
            UpdateStateMachine();
            const double timeBeforeTree = ros::Time::now().toSec();

            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
            behaviorTree->Tick();
            std::chrono::time_point<std::chrono::system_clock> foo = std::chrono::system_clock::now();
            mTreeTimeStep = ros::Time::now().toSec() - timeBeforeTree;

            PubBehavior(behaviorTree);
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(foo - now);
            auto ms = milliseconds.count();

            std::cout << "LFM ms = " <<ms<< '\n';
        }

        #ifdef DEBUG_TREE
        {
            std_msgs::String message;
            message.data = behaviorTreeGraphBuilder.produceDot();
            behaviorTreeGraphPublisher.publish(message);
        }
        #endif // DEBUG_TREE

        loop_rate.sleep();
        mContext->debug->StopAddDebug();
    }
}

}/* namespace Behavior */
