#ifndef POSE_SMOOTHER_NODE_H
#define POSE_SMOOTHER_NODE_H

#include <vector>
#include <queue>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

struct Pose
{
    double x;
    double y;
    double z;
};

class PoseSmootherNode
{
public:
    PoseSmootherNode();
    void Run();

private:
    ros::NodeHandle mNodeHandle;
    ros::Subscriber mPoseSubscriber;
    ros::Publisher mPoseSmoothedPublisher;
    ros::Publisher mTimeSmoothingPublisher;

    std::queue<Pose> mPoseQueue;
    unsigned int mPoseQueueMaxSize;
    Pose mPoseQueueSum;

    ros::Time mSmoothingStart;
    ros::Time mSmoothingEnd;

    void PoseCallback(const geometry_msgs::PoseStamped &pose);
};

#endif // POSE_SMOOTHER_NODE_H
