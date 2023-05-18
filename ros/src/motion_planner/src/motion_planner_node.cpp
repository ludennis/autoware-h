#include <motion_planner/along_global_path_state.h>
#include <motion_planner/cost_function.h>
#include <motion_planner/motion_planner_node.h>
#include <motion_planner/moving_range_matching.h>
#include <motion_planner/state.h>

static const int NODE_RATE = 100;
static const int TOPIC_BUFFER = 1;
static const double DEFAULT_ZERO = 0.0;
static const double WAYPOINT_RES_DEFAULT = 1.0;
static const double ROLL_OUT_DIST_DEFAULT = 50.0;
static const double ROLL_IN_FACTOR_DEFAULT = 5.0;

static void InitWaypointMarker(visualization_msgs::Marker & marker)
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points_and_lines";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 1.9;
    marker.color.b = 0.2;
    marker.color.g = 0.8;
    marker.color.a = 0.5;
}

MotionPlannerNode::MotionPlannerNode()
    : mNodeHandle()
    , mSubCarState()
    , mSubBehavior()
    , mSubObjList()
    , mSubParkingStatus()
    , mSubVehicleState()
    , mRate(NODE_RATE)
    , mContext()
{
    mContext.mPubWaypoints = mNodeHandle.advertise<itri_msgs::WaypointArray>(
        "/waypoints", TOPIC_BUFFER);
    mContext.mPubWaypointsRviz =
        mNodeHandle.advertise<visualization_msgs::Marker>(
            "/rviz_waypoints", TOPIC_BUFFER);
    mContext.mPubLocalPaths = mNodeHandle.advertise<itri_msgs::WaypointArrays>(
        "/local_paths", TOPIC_BUFFER);

    mSubCarState = mNodeHandle.subscribe("/car_state", TOPIC_BUFFER,
        &MotionPlannerNode::CallbackCarState, this);
    mSubBehavior = mNodeHandle.subscribe("/behavior", TOPIC_BUFFER,
        &MotionPlannerNode::CallbackBehavior, this);
    mSubObjList = mNodeHandle.subscribe("/detected_objects", TOPIC_BUFFER,
        &MotionPlannerNode::CallbackObjList, this);
    mSubGlobalPath = mNodeHandle.subscribe("/global_path", TOPIC_BUFFER,
        &MotionPlannerNode::CallbackGlobalPath, this);
    mSubVehicleState = mNodeHandle.subscribe("/vehicle_state", TOPIC_BUFFER,
        &MotionPlannerNode::CallbackVehicleState, this);

    ros::NodeHandle privateNode("~");
    privateNode.param<double>(
        "weight_cen_cost",
        mContext.mPlannerParam.weightCenter, DEFAULT_ZERO);
    privateNode.param<double>(
        "weight_tran_cost",
        mContext.mPlannerParam.weightTransition, DEFAULT_ZERO);
    privateNode.param<double>(
        "weight_col_cost",
        mContext.mPlannerParam.weightCollision, DEFAULT_ZERO);
    privateNode.param<double>(
        "waypt_resolution",
        mContext.mPlannerParam.wayptRes, WAYPOINT_RES_DEFAULT);
    privateNode.param<double>(
        "roll_out_dist",
        mContext.mPlannerParam.rollOutDist, ROLL_OUT_DIST_DEFAULT);
    privateNode.param<double>(
        "roll_in_factor",
        mContext.mPlannerParam.rollInFactor, ROLL_IN_FACTOR_DEFAULT);

    InitWaypointMarker(mContext.mWaypointMarker);
}

void MotionPlannerNode::MainLoop()
{
    MovingRangeMatching pathMatch;

    std::shared_ptr<State> localPathsState =
        std::make_shared<AlongGlobalPathState>();

    while (ros::ok())
    {
        ros::spinOnce();

        if (mContext.mGlobalPath &&
            mContext.mCarState.is_stable)
        {
            const int indexMatch = pathMatch.RunOnce(
                mContext.mGlobalPath->mResolution, mContext.mGlobalPath->mPath,
                mContext.mCarState);

            ROS_INFO("%s", localPathsState->GetClassName().c_str());
            localPathsState = localPathsState->Generate(
                indexMatch, mContext);
        }
        mRate.sleep();
    }
}


void MotionPlannerNode::CallbackCarState(
    const itri_msgs::CarState & msg)
{
    mContext.mCarState = msg;
}

void MotionPlannerNode::CallbackBehavior(
    const itri_msgs::plan & msg)
{
    mContext.mBehavior.hint = msg.hint;
    if (!mContext.mBehavior.hint)
        return;

    if (msg.update_num_level && msg.num_level > 0)
        mContext.mBehavior.num_level = msg.num_level;
    if (msg.update_dev && msg.dev > 0.0f)
        mContext.mBehavior.dev = msg.dev;
    if (msg.update_bias)
        mContext.mBehavior.bias = msg.bias;

    mContext.mBehavior.parking_start = msg.parking_start;
}

void MotionPlannerNode::CallbackObjList(
    const itri_msgs::DetectedObjectArray & msg)
{
    mContext.mObjectList.objects.clear();
    for (const auto & object : msg.objects)
    {
        itri_msgs::DetectedObject itriObject;
        itriObject.id = object.id;
        itriObject.label = object.label;
        itriObject.pose = object.pose;
        itriObject.dimensions = object.dimensions;
        itriObject.variance = object.variance;
        itriObject.velocity = object.velocity;
        itriObject.pose_reliable = object.pose_reliable;
        itriObject.velocity_reliable = object.velocity_reliable;
        itriObject.behavior_state = object.behavior_state;

        for (const auto & point : object.convex_hull.polygon.points)
        {
            itri_msgs::PointXYZSD itriPoint;
            itriPoint.x = point.x;
            itriPoint.y = point.y;
            itriPoint.z = point.z;
            itriObject.convex_hull.polygon.points.push_back(itriPoint);
        }

        mContext.mObjectList.objects.push_back(itriObject);
    }
    ObjectTransform();
}

void MotionPlannerNode::CallbackGlobalPath(const itri_msgs::Path & msg)
{
    mContext.mGlobalPath = std::make_shared<GlobalPath>(msg);
}

void MotionPlannerNode::CallbackVehicleState(
    const itri_msgs::VehicleState & msg)
{
    mContext.mVehicleState = msg;
}

void MotionPlannerNode::ObjectTransform()
{
    try
    {
        mContext.mObjectTransformListener.lookupTransform(
            "/map", "/base_link", ros::Time(0), mContext.mObjectTransform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }

    for (auto & object : mContext.mObjectList.objects)
    {
        for (auto & point : object.convex_hull.polygon.points)
        {
            tf::Point tfPoint(point.x, point.y, point.z);
            tfPoint = mContext.mObjectTransform * tfPoint;
            point.x = tfPoint.x();
            point.y = tfPoint.y();
            point.z = tfPoint.z();
        }
    }
}
