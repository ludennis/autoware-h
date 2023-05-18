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

#include <state_estimator/state_estimator.h>

constexpr int kImuFrequencyHz = 100;
const char* kFilename =
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__);

namespace
{

double DistanceFromTimestampedPoses(
  const common::TimestampedPose &pose1, const common::TimestampedPose &pose2)
{
  return std::sqrt(
    std::pow(pose1.x - pose2.x, 2.0) +
    std::pow(pose1.y - pose2.y, 2.0) +
    std::pow(pose1.z - pose2.z, 2.0));
}

double SpeedFromTimestampedPoses(
  const common::TimestampedPose &pose1, const common::TimestampedPose &pose2)
{
  return DistanceFromTimestampedPoses(pose1, pose2) /
    (std::abs(pose1.timestamp - pose2.timestamp));
}

std::vector<sensor_msgs::Imu> GetImuWithinDuration(
  const std::deque<sensor_msgs::Imu>& imuDeque,
  const double startTime,
  const double deltaTime,
  unsigned int &outputStartIndex)
{
  unsigned int imuDequeStartIndex =
    (startTime - imuDeque.front().header.stamp.toSec()) / (1.0 / kImuFrequencyHz);
  unsigned int imuDequeEndIndex =
    imuDequeStartIndex + std::floor(deltaTime /  (1.0 / kImuFrequencyHz));
  outputStartIndex = imuDequeStartIndex;

  std::vector<sensor_msgs::Imu> imuWithinDuration;
  if (imuDequeEndIndex >= imuDeque.size())
  {
    imuDequeEndIndex = imuDeque.size() - 1;
  }
  for (unsigned int i = imuDequeStartIndex; i <= imuDequeEndIndex; ++i)
  {
    imuWithinDuration.push_back(imuDeque.at(i));
  }

  if (imuWithinDuration.size() == 0)
  {
    imuWithinDuration.push_back(imuDeque.back());
    outputStartIndex = imuDeque.size() - 1;
  }

  return imuWithinDuration;
}

} // namespace

StateEstimator::StateEstimator()
{}

common::TimestampedPose StateEstimator::EstimatePose(
  bool useImuData, bool useOdometerData,
  const common::TimestampedPose &latestPose,
  const double startTime, const double deltaTime,
  const double poseTime)
{
  unsigned int imuWithinDurationStartIndex = 0;
  auto imuWithinDuration = GetImuWithinDuration(mImuDeque,
    startTime, deltaTime, imuWithinDurationStartIndex);

  ROS_DEBUG_STREAM("imuWithinDuration.size(): " << imuWithinDuration.size());

  if (mScanMatchedPoseDeque.size() <= 0)
  {
    std::stringstream ss;
    ss << "[" << kFilename << ":" << __LINE__ << "] mScanMatchedPoseDeque is empty!";
    throw std::runtime_error(ss.str());
  }

  double diffTime = deltaTime;
  double diffOffsetX = 0.0;
  double diffOffsetY = 0.0;
  double diffOffsetZ = 0.0;
  double diffOffsetYaw = 0.0;
  common::TimestampedPose latestScanMatchedPose = mScanMatchedPoseDeque.back();
  sensor_msgs::Imu latestImu;
  double latestVehicleSpeed = 0.0;

  if (useImuData && !mImuDeque.empty())
  {
    latestImu = mImuDeque.back();
  }
  else if (!useImuData)
  {
    ROS_ERROR("[%s:%d] mImuDeque is empty! Please check imu.",
      kFilename, __LINE__);
  }

  if (useOdometerData && !mVehicleStatusDeque.empty())
  {
    latestVehicleSpeed = mVehicleStatusDeque.back().velocity.norm;
  }
  else if (useOdometerData && mVehicleStatusDeque.empty())
  {
    ROS_ERROR("[%s:%d] mVehicleSpeedDeque is empty! Please check vehicle state.",
      kFilename, __LINE__);
  }

  if (mScanMatchedPoseDeque.size() == 1u)
  {
    return mScanMatchedPoseDeque.back();
  }

  if (mScanMatchedPoseDeque.size() >= 2u && diffTime == 0.0)
  {
    common::TimestampedPose secondLastScanMatchedPose =
      mScanMatchedPoseDeque.at(mScanMatchedPoseDeque.size()-2);
    diffTime = startTime - latestScanMatchedPose.timestamp;
    diffOffsetX = latestScanMatchedPose.x - secondLastScanMatchedPose.x;
    diffOffsetY = latestScanMatchedPose.y - secondLastScanMatchedPose.y;
    diffOffsetZ = latestScanMatchedPose.z - secondLastScanMatchedPose.z;
    diffOffsetYaw = latestScanMatchedPose.yaw - secondLastScanMatchedPose.yaw;
  }

  common::TimestampedPose estimatedPose = latestPose;
  double localOffsetX = 0.0;
  double localOffsetY = 0.0;
  double localOffsetZ = 0.0;
  double localOffsetRoll = 0.0;
  double localOffsetPitch = 0.0;
  double localOffsetYaw = 0.0;
  double velocityX = diffOffsetX / diffTime;
  double velocityY = diffOffsetY / diffTime;
  double velocityZ = diffOffsetZ / diffTime;

  if (useImuData && useOdometerData)
  {
    double yawRateTheshold = 0.01;
    double yawAngularAcceleration = 0.0;
    double vehicleAcceleration = 0.0;

    auto prevImu = imuWithinDuration[0];
    if ((imuWithinDurationStartIndex-1) > 0)
    {
      prevImu = mImuDeque[imuWithinDurationStartIndex-1];
    }
    else
    {
      diffTime = 0.0;
    }

    for (auto imu : imuWithinDuration)
    {
      if (imu.header.stamp != prevImu.header.stamp)
      {
        diffTime = (imu.header.stamp - prevImu.header.stamp).toSec();
      }

      if (diffTime > 0)
      {
        yawAngularAcceleration =
          (imu.angular_velocity.z - prevImu.angular_velocity.z) / diffTime;
        vehicleAcceleration = (latestVehicleSpeed -
          mVehicleStatusDeque[mVehicleStatusDeque.size() - 2].velocity.norm)
          / diffTime;

        if (std::fabs(imu.angular_velocity.z) < yawRateTheshold)
        {
          // on the straight
          const auto pitch = estimatedPose.pitch + imu.angular_velocity.y * diffTime;
          const auto diffDistance = (latestVehicleSpeed) * diffTime;

          localOffsetX = diffDistance * cos(estimatedPose.yaw) +
            cos(estimatedPose.yaw) * vehicleAcceleration * std::pow(diffTime, 2) / 2;
          localOffsetY = diffDistance * sin(estimatedPose.yaw) +
            sin(estimatedPose.yaw) * vehicleAcceleration * std::pow(diffTime, 2) / 2;
          localOffsetZ = diffDistance * sin(-pitch);

          localOffsetRoll = imu.angular_velocity.x * diffTime;
          localOffsetPitch = imu.angular_velocity.y * diffTime;
          localOffsetYaw = imu.angular_velocity.z * diffTime +
            yawAngularAcceleration * std::pow(diffTime, 2) / 2;
        }
        else
        {
          const auto velocityYawRate =
            (latestVehicleSpeed) / imu.angular_velocity.z;

          const auto pitch = estimatedPose.pitch + imu.angular_velocity.y * diffTime;
          const auto yaw = estimatedPose.yaw + imu.angular_velocity.z * diffTime;
          const auto diffDistance = (latestVehicleSpeed) * diffTime;

          localOffsetX = velocityYawRate * (sin(yaw) - sin(estimatedPose.yaw)) +
            cos(estimatedPose.yaw) * vehicleAcceleration * std::pow(diffTime, 2) / 2;
          localOffsetY = velocityYawRate * (-cos(yaw) + cos(estimatedPose.yaw)) +
            sin(estimatedPose.yaw) * vehicleAcceleration * std::pow(diffTime, 2) / 2;
          localOffsetZ = diffDistance * sin(-pitch);

          localOffsetRoll = imu.angular_velocity.x * diffTime;
          localOffsetPitch = imu.angular_velocity.y * diffTime;
          localOffsetYaw = imu.angular_velocity.z * diffTime +
            yawAngularAcceleration * std::pow(diffTime, 2) / 2;

        }
        estimatedPose.x = estimatedPose.x + localOffsetX;
        estimatedPose.y = estimatedPose.y + localOffsetY;
        estimatedPose.z = estimatedPose.z + localOffsetZ;
        estimatedPose.roll = estimatedPose.roll + localOffsetRoll;
        estimatedPose.pitch = estimatedPose.pitch + localOffsetPitch;
        estimatedPose.yaw = estimatedPose.yaw + localOffsetYaw;
      }
      prevImu = imu;

    }
    estimatedPose.timestamp = poseTime;

    return estimatedPose;
  }
  else if (useImuData && !useOdometerData)
  {
    const auto roll = latestScanMatchedPose.roll + latestImu.angular_velocity.x * diffTime;
    const auto pitch = latestScanMatchedPose.pitch + latestImu.angular_velocity.y * diffTime;
    const auto yaw = latestScanMatchedPose.yaw + latestImu.angular_velocity.z * diffTime;

    const auto accX1 = latestImu.linear_acceleration.x;
    const auto accY1 = std::cos(roll) * latestImu.linear_acceleration.y -
                   std::sin(roll) * latestImu.linear_acceleration.z;
    const auto accZ1 = std::sin(roll) * latestImu.linear_acceleration.y +
                   std::cos(roll) * latestImu.linear_acceleration.z;

    const auto accX2 = std::sin(pitch) * accZ1 + std::cos(pitch) * accX1;
    const auto accY2 = accY1;
    const auto accZ2 = std::cos(pitch) * accZ1 - std::sin(pitch) * accX1;

    const auto accX = std::cos(yaw) * accX2 - std::sin(yaw) * accY2;
    const auto accY = std::sin(yaw) * accX2 + std::cos(yaw) * accY2;
    const auto accZ = accZ2;

    localOffsetX += velocityX * diffTime + accX * diffTime * diffTime / 2.0;
    localOffsetY += velocityY * diffTime + accY * diffTime * diffTime / 2.0;
    localOffsetZ += velocityZ * diffTime + accZ * diffTime * diffTime / 2.0;

    velocityX += accX * diffTime;
    velocityY += accY * diffTime;
    velocityZ += accZ * diffTime;

    localOffsetRoll += latestImu.angular_velocity.x * diffTime;
    localOffsetPitch += latestImu.angular_velocity.y * diffTime;
    localOffsetYaw += latestImu.angular_velocity.z * diffTime;
  }
  else if (!useImuData && !useOdometerData)
  {
    localOffsetX = diffOffsetX;
    localOffsetY = diffOffsetY;
    localOffsetZ = diffOffsetZ;
    localOffsetRoll = 0.0;
    localOffsetPitch = 0.0;
    localOffsetYaw = diffOffsetYaw;
  }

  estimatedPose.x = latestPose.x + localOffsetX;
  estimatedPose.y = latestPose.y + localOffsetY;
  estimatedPose.z = latestPose.z + localOffsetZ;
  estimatedPose.roll = latestPose.roll + localOffsetRoll;
  estimatedPose.pitch = latestPose.pitch + localOffsetPitch;
  estimatedPose.yaw = latestPose.yaw + localOffsetYaw;
  estimatedPose.timestamp = poseTime;

  return estimatedPose;
}

common::TimestampedVehicleStatus StateEstimator::EstimateVehicleStatus()
{
  double diffTime = 0.0;
  double diffOffsetX = 0.0;
  double diffOffsetY = 0.0;
  double diffOffsetZ = 0.0;
  double diffOffsetYaw = 0.0;
  double diffOffset = 0.0;
  common::TimestampedPose latestScanMatchedPose = mScanMatchedPoseDeque.back();

  if (mScanMatchedPoseDeque.size() == 1)
  {
    common::TimestampedVehicleStatus updateVehicleStatus;
    updateVehicleStatus.timestamp = latestScanMatchedPose.timestamp;
    return updateVehicleStatus;
  }

  if (mScanMatchedPoseDeque.size() >= 2 && diffTime == 0)
  {
    common::TimestampedPose secondLastScanMatchedPose =
      mScanMatchedPoseDeque.at(mScanMatchedPoseDeque.size()-2);
    diffTime = latestScanMatchedPose.timestamp - secondLastScanMatchedPose.timestamp;
    diffOffsetX = latestScanMatchedPose.x - secondLastScanMatchedPose.x;
    diffOffsetY = latestScanMatchedPose.y - secondLastScanMatchedPose.y;
    diffOffsetZ = latestScanMatchedPose.z - secondLastScanMatchedPose.z;
    diffOffsetYaw = latestScanMatchedPose.yaw - secondLastScanMatchedPose.yaw;
    diffOffset = sqrt(diffOffsetX * diffOffsetX +
      diffOffsetY * diffOffsetY + diffOffsetZ * diffOffsetZ);
  }

  common::Velocity estimatedVelocity;
  common::Acceleration estimatedAcceleration;

  if (diffTime == 0.0f)
  {
    estimatedVelocity.norm = 0.0f;
    estimatedVelocity.x = 0.0f;
    estimatedVelocity.y = 0.0f;
    estimatedVelocity.z = 0.0f;
    estimatedVelocity.angularVelocity = 0.0f;

    estimatedAcceleration.norm = 0.0f;
    estimatedAcceleration.x = 0.0f;
    estimatedAcceleration.y = 0.0f;
    estimatedAcceleration.z = 0.0f;
  }
  else
  {
    estimatedVelocity.norm = diffOffset / diffTime;
    estimatedVelocity.x = diffOffsetX / diffTime;
    estimatedVelocity.y = diffOffsetY / diffTime;
    estimatedVelocity.z = diffOffsetZ / diffTime;
    estimatedVelocity.angularVelocity = diffOffsetYaw / diffTime;

    common::TimestampedVehicleStatus latestVehicleStatus;
    if (!mEstimatedVehicleStatusDeque.empty())
    {
      latestVehicleStatus = mEstimatedVehicleStatusDeque.back();
    }
    estimatedAcceleration.norm =
      (estimatedVelocity.norm - latestVehicleStatus.velocity.norm) / diffTime;
    estimatedAcceleration.x =
      (estimatedVelocity.x - latestVehicleStatus.velocity.x) / diffTime;
    estimatedAcceleration.y =
      (estimatedVelocity.y - latestVehicleStatus.velocity.y) / diffTime;
    estimatedAcceleration.z =
      (estimatedVelocity.z - latestVehicleStatus.velocity.z) / diffTime;
  }

  common::TimestampedVehicleStatus updateVehicleStatus;
  updateVehicleStatus.timestamp = latestScanMatchedPose.timestamp;
  updateVehicleStatus.velocity = estimatedVelocity;
  updateVehicleStatus.acceleration = estimatedAcceleration;

  return updateVehicleStatus;
}

void StateEstimator::PushEstimatedPose(
  const common::TimestampedPose &estimatedPose)
{
  if (mEstimatedPoseDeque.size() >= 1 &&
      estimatedPose == mEstimatedPoseDeque.back())
  {
    return;
  }
  if (mEstimatedPoseDeque.size() >= kDequeSize)
  {
    mEstimatedPoseDeque.pop_front();
  }
  mEstimatedPoseDeque.push_back(estimatedPose);
}

void StateEstimator::PushImuData(const sensor_msgs::Imu &imu)
{
  if (mImuDeque.size() >= kDequeSize)
  {
    mImuDeque.pop_front();
  }
  mImuDeque.push_back(imu);
}

unsigned int StateEstimator::GetVehicleStatusDequeSize()
{
  return mVehicleStatusDeque.size();
}

void StateEstimator::PushVehicleStatus(
  const common::TimestampedVehicleStatus &vehicleStatus)
{
  if (mVehicleStatusDeque.size() >= kDequeSize)
  {
    mVehicleStatusDeque.pop_front();
  }
  mVehicleStatusDeque.push_back(vehicleStatus);
}

void StateEstimator::PushEstimatedVehicleStatus(
  const common::TimestampedVehicleStatus &etimatedVehicleStatus)
{
  if (mEstimatedVehicleStatusDeque.size() >= kDequeSize)
  {
    mEstimatedVehicleStatusDeque.pop_front();
  }
  mEstimatedVehicleStatusDeque.push_back(etimatedVehicleStatus);
}

void StateEstimator::PushScanMatchedPose(
  const common::TimestampedPose &scanMatchedPose)
{
  if (mScanMatchedPoseDeque.size() >= kDequeSize)
  {
    mScanMatchedPoseDeque.pop_front();
  }
  mScanMatchedPoseDeque.push_back(scanMatchedPose);
}

void StateEstimator::PushGnssPose(const common::TimestampedPose &gnssPose)
{
  if (mGnssPoseDeque.size() >= 1 && (gnssPose == mGnssPoseDeque.back() ||
      DistanceFromTimestampedPoses(
        gnssPose, mGnssPoseDeque.back()) == 0))
  {
    return;
  }
  if (mGnssPoseDeque.size() >= kDequeSize)
  {
    mGnssPoseDeque.pop_front();
  }
  mGnssPoseDeque.push_back(gnssPose);
}

common::TimestampedPose StateEstimator::GetLatestEstimatedPose()
{
  assert("mEstimatedPoseDeque.empty()" && !mEstimatedPoseDeque.empty());
  return mEstimatedPoseDeque.back();
}

common::TimestampedPose StateEstimator::GetLatestGnssPose()
{
  assert("mGnssPoseDeque.empty()" && !mGnssPoseDeque.empty());
  return mGnssPoseDeque.back();
}

common::TimestampedPose StateEstimator::GetLatestScanMatchedPose()
{
  assert("mScanMatchedPoseDeque.empty()" && !mScanMatchedPoseDeque.empty());
  return mScanMatchedPoseDeque.back();
}

unsigned int StateEstimator::GetGnssPoseDequeSize()
{
  return mGnssPoseDeque.size();
}

common::TimestampedVehicleStatus StateEstimator::GetLatestVehicleStatus()
{
  if (mVehicleStatusDeque.empty())
  {
    return common::TimestampedVehicleStatus();
  }
  return mVehicleStatusDeque.back();
}

common::TimestampedVehicleStatus StateEstimator::GetLatestEstimatedVehicleStatus()
{
  if (mEstimatedVehicleStatusDeque.size() <= 0)
  {
    return common::TimestampedVehicleStatus();
  }
  return mEstimatedVehicleStatusDeque.back();
}

unsigned int StateEstimator::GetScanMatchedPoseDequeSize()
{
  return mScanMatchedPoseDeque.size();
}

unsigned int StateEstimator::GetEstimatedVehicleStatusDequeSize()
{
  return mEstimatedVehicleStatusDeque.size();
}

unsigned int StateEstimator::GetEstimatedPoseDequeSize()
{
  return mEstimatedPoseDeque.size();
}

void StateEstimator::ClearScanMatchedPoseDeque()
{
  mScanMatchedPoseDeque.clear();
}

void StateEstimator::ClearGnssPoseDeque()
{
  mGnssPoseDeque.clear();
}

float StateEstimator::GetSpeedKmphFromGnssPoseDeque()
{
  return GetSpeedFromGnssPoseDeque() * 3.6;
}

float StateEstimator::GetSpeedFromGnssPoseDeque()
{
  if (mGnssPoseDeque.size() < 2)
  {
    return 0.0;
  }
  return SpeedFromTimestampedPoses(
    *(mGnssPoseDeque.cend() - 2), mGnssPoseDeque.back());
}

float StateEstimator::GetDistanceFromGnssPoseDeque()
{
  assert("mGnssPoseDeque.size()" && mGnssPoseDeque.size() >= 2);
  return DistanceFromTimestampedPoses(
    *(mGnssPoseDeque.cend() - 2), mGnssPoseDeque.back());
}

std::vector<common::Pose> StateEstimator::GetAllScanMatchedPoses()
{
  std::vector<common::Pose> poses;
  for (auto pose : mScanMatchedPoseDeque)
  {
    poses.push_back(pose);
  }
  return poses;
}

std::vector<common::Pose> StateEstimator::GetAllEstimatedPoses()
{
  std::vector<common::Pose> poses;
  for (auto pose : mEstimatedPoseDeque)
  {
    poses.push_back(pose);
  }
  return poses;
}

void StateEstimator::ClearImuDeque()
{
  mImuDeque.clear();
}

void StateEstimator::ClearVehicleStatusDeque()
{
  mVehicleStatusDeque.clear();
}

void StateEstimator::ClearEstimatedVehicleStatusDeque()
{
  mEstimatedVehicleStatusDeque.clear();
}

void StateEstimator::ClearDeques()
{
  ClearScanMatchedPoseDeque();
  ClearGnssPoseDeque();
  ClearImuDeque();
  ClearVehicleStatusDeque();
  ClearEstimatedVehicleStatusDeque();
}
