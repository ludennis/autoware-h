#ifndef __VEHICLE_TRUCK_GATEWAY_NODE_H__
#define __VEHICLE_TRUCK_GATEWAY_NODE_H__

#include <can_msgs/Frame.h>
#include <eps_controller.h>
#include <eps_diagnosis.h>
#include <memory>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <itri_msgs/blinker_cmd.h>
#include <itri_msgs/speed_cmd.h>
#include <itri_msgs/steer_cmd.h>
#include <itri_msgs/VehicleState.h>
#include <vcu_truck_message_data.h>

class VehicleGateway
{
public:
    VehicleGateway();
    float GetLimitedSteerCmd(const float originalSteerCommand);
    void BlinkerCmdCallback(const itri_msgs::blinker_cmd &);
    void SpeedCmdCallback(const itri_msgs::speed_cmd &);
    void SteerCmdCallback(const itri_msgs::steer_cmd &);
    void GetSelfDrivingStatus();
    void GetGearControlCmd();
    void GetTruckGearRatio();
    void GetCruiseControlCmd();
    void GetTruckControlCmd();
    void GetTargetEngineSpeed();
    void IpcRollingCounterCallback();
    void TruckControlCmdCallback();
    void EngineControlCmdCallback();
    void OriginalMessagesCallback();
    void ReceivedMessagesCallback(const can_msgs::Frame &);
    void RunOnce();

private:
    ros::NodeHandle mNodeHandle;
    ros::Subscriber mBlinkerCmdSubscriber;
    ros::Subscriber mSpeedCmdSubscriber;
    ros::Subscriber mSteerCmdSubscriber;
    ros::Publisher mSentMessagesPublisher;
    ros::Subscriber mReceivedMessagesSubscriber;
    ros::Publisher mVehicleDbwStatePublisher;
    ros::Rate mRate;
    itri_msgs::VehicleState mVehicleDbwState;
    ControlCmdRaw mSpeedCmd;
    ControlCmdRaw mSteeringCmd;
    ControlCmdRaw1 mTruckControlCmd;
    ControlCmdRaw2 mIpcKeepAlive;
    ControlCmdRaw3 mEngineControlCmd;
    float mSteerLimitESC;
    float mTruckGearRatio;
    float mTargetEngineSpd;
    bool mSelfDrivingStatus;
    bool mResetSelfDrivingStatus;
    int mGatewayCounter;
    std::shared_ptr<EPSController> mEpsControllerRight;
    std::shared_ptr<EPSController> mEpsControllerLeft;
    EPSDiagnosis mEPSDiagnosis;
};

#endif // __VEHICLE_TRUCK_GATEWAY_NODE_H__
