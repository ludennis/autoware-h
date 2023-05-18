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

/*
 * Function EstimatePose() took reference from functions imu_odom_calc,
 * odom_calc(), imu_calc() from file:
 * https://github.com/autowarefoundation/autoware/blob/release/
 *  1.8.1/ros/src/computing/perception/localization/
 *  packages/lidar_localizer/nodes/ndt_matching/ndt_matching.cpp
 */

#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <common/pose.h>
#include <common/vehicle_status.h>
#include <deque>
#include <sensor_msgs/Imu.h>
#include <sstream>

auto constexpr kDequeSize = 500u;

/*
 * The StateEstimator estimates the current state, or pose,
 * of the vehicle with information from scan-matched pose, IMU, and odometry.
 */
class StateEstimator
{
public:
  StateEstimator();
  /*
   * Estimate a future pose that has traveled over time interval, delta time,
   * given the starting pose
   */
  common::TimestampedPose EstimatePose(bool useImuData, bool useOdometerData,
    const common::TimestampedPose &latestPose,
    const double startTime, const double deltaTime,
    const double poseTime);
  common::TimestampedVehicleStatus EstimateVehicleStatus();

  void PushEstimatedPose(const common::TimestampedPose &estimatedPose);
  void PushScanMatchedPose(const common::TimestampedPose &scanMatchedPose);
  void PushGnssPose(const common::TimestampedPose &gnssPose);
  void PushImuData(const sensor_msgs::Imu &imu);
  void PushVehicleStatus(const common::TimestampedVehicleStatus &vehicleStatus);
  void PushEstimatedVehicleStatus(
    const common::TimestampedVehicleStatus &estimatedVehicleStatus);

  common::TimestampedPose GetLatestEstimatedPose();
  common::TimestampedPose GetLatestGnssPose();
  common::TimestampedPose GetLatestScanMatchedPose();
  common::TimestampedVehicleStatus GetLatestVehicleStatus();
  common::TimestampedVehicleStatus GetLatestEstimatedVehicleStatus();

  unsigned int GetScanMatchedPoseDequeSize();
  unsigned int GetVehicleStatusDequeSize();
  unsigned int GetEstimatedVehicleStatusDequeSize();
  unsigned int GetGnssPoseDequeSize();
  unsigned int GetEstimatedPoseDequeSize();

  float GetSpeedFromGnssPoseDeque();
  float GetSpeedKmphFromGnssPoseDeque();
  float GetDistanceFromGnssPoseDeque();

  std::vector<common::Pose> GetAllScanMatchedPoses();
  std::vector<common::Pose> GetAllEstimatedPoses();

  void ClearDeques();

  void ClearScanMatchedPoseDeque();
  void ClearGnssPoseDeque();
  void ClearImuDeque();
  void ClearVehicleStatusDeque();
  void ClearEstimatedVehicleStatusDeque();

private:
  std::deque<common::TimestampedPose> mEstimatedPoseDeque;
  std::deque<common::TimestampedPose> mScanMatchedPoseDeque;
  std::deque<common::TimestampedPose> mGnssPoseDeque;
  std::deque<sensor_msgs::Imu> mImuDeque;
  std::deque<common::TimestampedVehicleStatus> mVehicleStatusDeque;
  std::deque<common::TimestampedVehicleStatus> mEstimatedVehicleStatusDeque;
};

#endif //STATE_ESTIMATOR_H
