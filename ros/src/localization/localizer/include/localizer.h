/*
 * Copyright (c) 2014, Nagoya University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Autoware nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ndt_matcher/ndt_matcher.h>
#include <state_estimator/state_estimator.h>

#include <chrono>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <itri_msgs/NdtStatistics.h>
#include <itri_msgs/VehicleState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


/*
 * Class Localizer is the top-level class to encapsulate some localization-related
 * subclasses, handling input sensors first-handedly and responsible for the overall
 * control flow. Reasonably, all ROS Subscribers, Publishers, and Callback functions
 * are implemented within this class to help dispatch related inputs to responsible
 * subclasses.
 *
 * Refactored class based on the structure of file:
 * https://github.com/autowarefoundation/autoware/blob/
 *  release/1.8.1/ros/src/computing/perception/localization/
 *  packages/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp
 */
class Localizer
{
public:
  Localizer();

private:
  void GetParameters();
  Eigen::Matrix4f InitTfLidarToBaselink(
    const double tfX, const double tfY, const double tfZ,
    const double tfRoll, const double tfPitch, const double tfYaw);
  void PublishTfBaselinkToMap(ros::Time &currentTime, common::Pose &inputPose);
  void PublishNdtState(ros::Time currentTime, double executionTime, int iteration,
    unsigned int validPointsNumber, unsigned int invalidPointsNumber,
    unsigned int sourcePointsNumber, double fitnessScore,
    double fitnessScoreWithRadiusSearch, double ndtMatchingScore,
    double ndtMatchingScoreWithValidPoints, double estimatedVelocity,
    double estimatedAcceleration);

  void ThreadFunction();

  void MapCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
  void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
  void GnssCallback(const geometry_msgs::PoseStamped::ConstPtr& input);
  void InitialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input);
  void VehicleStateCallback(const itri_msgs::VehicleState& input);
  void ImuCallback(const sensor_msgs::Imu::Ptr& input);

  bool HasGoodScanMatchingResult();
  bool HasEnoughValidPoints();
  bool HasHighPositionalErrorAgainstGnssPose();

  /* updates mTfX mTfY mTfZ mTfRoll mTfPitch mTfYaw */
  void WaitForRangefinderToBaseLinkTransform();

  void Reset();
  void SetInitialPose(const common::TimestampedPose& pose);
  void SetInitialPose(const geometry_msgs::Pose& pose, const std::string& frame,
    const std::string& targetFrame, const ros::Time& timestamp);

  ros::NodeHandle mNh;
  ros::NodeHandle mPrivateNh;

  ros::Subscriber mSubscriberPoints;
  ros::Subscriber mSubscriberGnssPose;
  ros::Subscriber mSubscriberInitialPose;
  ros::Subscriber mSubscriberVehicleState;
  ros::Subscriber mSubscriberImu;

  ros::Publisher mPublisherNdtPose;
  ros::Publisher mPublisherPredictPose;
  ros::Publisher mPublisherNdtState;
  ros::Publisher mNdtValidPointsPub;
  ros::Publisher mNdtInvalidPointsPub;
  ros::Publisher mNdtPosesMarkerPublisher;
  ros::Publisher mEstimatedPosesMarkerPublisher;

  std::thread mMapUpdatingThread;

  tf::TransformBroadcaster mBrBaselinkMap;

  unsigned int mQueueSize;
  unsigned int mMapPointsNum;
  int mNdtMaxIteration;
  double mNdtStepSize;
  double mNdtTransEpsilon;
  double mNdtOutlierRatio;
  float mNdtResolution;
  double mTfX;
  double mTfY;
  double mTfZ;
  double mTfYaw;
  double mTfPitch;
  double mTfRoll;
  double mFitnessScore;
  double mFitnessScoreWithValidPoints;
  double mFitnessScoreThreshold;
  double mNdtMatchingScore;
  double mNdtMatchingScoreWithValidPoints;
  double mNdtMatchingScoreThreshold;
  float mMotionFilterSpeedKmph;
  Eigen::Matrix4f mLidarToBaselinkTransformMatrix;
  Eigen::Matrix4f mBaselinkToMapTransformMatrix;
  std::string mImuTopic;
  std::mutex mMutex;
  bool mUseGpu;
  bool mUseImuData;
  bool mUseOdometerData;
  bool mUseGnss;
  bool mMapLoaded;
  bool mInitialPoseSet;
  std::string mRangefinderFrame;
  unsigned int mNumberValidPoints;
  unsigned int mNumberSourcePoints;

  common::TimestampedPose mInitialPose;

  std::shared_ptr<NdtMatcher> mNdt;
  StateEstimator mStateEstimator;
};
