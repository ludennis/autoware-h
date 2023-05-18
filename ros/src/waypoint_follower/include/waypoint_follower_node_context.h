#ifndef __WAYPOINT_FOLLOWER_NODE_CONTEXT_H__
#define __WAYPOINT_FOLLOWER_NODE_CONTEXT_H__

#include <global_path.h>
#include <itri_msgs/CarState.h>
#include <itri_msgs/WaypointArray.h>
#include <itri_msgs/Path.h>
#include <itri_msgs/speed_cmd.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <itri_msgs/VehicleState.h>

enum class ParkingPhase
{
    INITIAL = 0,
    PHASE1,
    PHASE2,
    PHASE3,
    FINISH,
};

struct ParkingStatus
{
    int parkingSpaceId;
    ParkingPhase phase;
};

class WaypointFollowerNodeContext
{
public:
    WaypointFollowerNodeContext();
    void SpinOnce();

protected:
    void CallbackCarState(const itri_msgs::CarState & msg);
    void CallbackWaypoint(const itri_msgs::WaypointArray & msg);
    void CallbackSpeedCmd(const itri_msgs::speed_cmd & msg);
    void CallbackVehicleState(const itri_msgs::VehicleState &);
    void CallbackGlobalPath(const itri_msgs::Path & msg);

public:
    ros::NodeHandle mNode;
    ros::Subscriber mSubCarState;
    ros::Subscriber mSubWaypoints;
    ros::Subscriber mSubSpeedCmd;
    ros::Subscriber mSubParkingStatus;
    ros::Subscriber mSubVehicleState;
    ros::Subscriber mSubGlobalPath;
    ros::Rate mRate;

    itri_msgs::CarState mCarState;
    itri_msgs::VehicleState mVehicleState;
    itri_msgs::WaypointArray mWaypoints;
    float mSpeedCmd;
    ParkingStatus mParkingStatus;
    std::shared_ptr<GlobalPath> mGlobalPath;

    std::string mControllerFileDir;

    double mParamGearRatio;
    double mParamPredictAheadTime;
    double mParamSteerOffset;
    double mParamWayptRes;
};

#endif // __WAYPOINT_FOLLOWER_NODE_CONTEXT_H__
