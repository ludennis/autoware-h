#include <string>

#include <itri_msgs/NdtStatistics.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <itri_msgs/CarState.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <itri_msgs/VehicleState.h>

enum class UpdateMode : uint8_t
{
    ALL = 0U,
    XY,
    YAW,
    NONE,
};

class Ndt2VehNode
{
public:
    Ndt2VehNode();
    void MainLoop();
    UpdateMode GetSensorUpdateMode(
        const geometry_msgs::PoseStamped & poseNew,
        geometry_msgs::PoseStamped & poseOld);

protected:
    void CallbackCarPose(const geometry_msgs::PoseStamped & msg);
    void CallbackCarTwist(const geometry_msgs::TwistStamped & msg);
    void CallbackCarYawRate(const sensor_msgs::Imu & msg);
    void CallbackCarSpeed(const itri_msgs::VehicleState & msg);
    void CallbackNdtStat(const itri_msgs::NdtStatistics & msg);

    void QuaternionToEulerAngles(geometry_msgs::PoseStamped & pose);

private:
    ros::NodeHandle mNode;
    ros::Publisher mPubCarState;
    ros::Subscriber mSubCarPose;
    ros::Subscriber mSubCarTwist;
    ros::Subscriber mSubYawRate;
    ros::Subscriber mSubSpeed;
    ros::Subscriber mSubNdtStat;

    geometry_msgs::PoseStamped mCarPose;
    geometry_msgs::PoseStamped mCarPoseOld;
    geometry_msgs::TwistStamped mCarTwist;
    double mYawRate;
    double mSpeed;
    itri_msgs::CarState mCarState;
    float mNdtScore;
    std::vector<float> mNdtScoreHistory;

    std::string mSubCarPoseTopic;
    double mParamNdtThreshold;
    double mParamModelCovariance[3];
    double mParamSensorCovariance[3];
};
