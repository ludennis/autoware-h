#ifndef POSE_H
#define POSE_H

#include <Eigen/Core>
#include <tf/tf.h>

namespace common
{

struct Pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  Pose()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , roll(0.0)
    , pitch(0.0)
    , yaw(0.0)
  {}

  Pose(Eigen::Matrix4f &eigenMatrix)
  {
    tf::Matrix3x3 tfMatrix;
    tfMatrix.setValue(
      static_cast<double>(eigenMatrix(0, 0)),
      static_cast<double>(eigenMatrix(0, 1)),
      static_cast<double>(eigenMatrix(0, 2)),
      static_cast<double>(eigenMatrix(1, 0)),
      static_cast<double>(eigenMatrix(1, 1)),
      static_cast<double>(eigenMatrix(1, 2)),
      static_cast<double>(eigenMatrix(2, 0)),
      static_cast<double>(eigenMatrix(2, 1)),
      static_cast<double>(eigenMatrix(2, 2)));

    x = eigenMatrix(0, 3);
    y = eigenMatrix(1, 3);
    z = eigenMatrix(2, 3);
    tfMatrix.getRPY(roll, pitch, yaw, 1);
  }

  Pose& operator=(const Pose& other)
  {
    x = other.x;
    y = other.y;
    z = other.z;
    roll = other.roll;
    pitch = other.pitch;
    yaw = other.yaw;
    return *this;
  }

  friend bool operator==(const Pose lhs, const Pose rhs)
  {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z &&
           lhs.roll == rhs.roll && lhs.pitch == rhs.pitch && lhs.yaw == rhs.yaw;
  }

  friend std::ostream& operator<<(std::ostream& outStream, const Pose& pose)
  {
    outStream << "Pose([x=" << pose.x << ", y=" << pose.y << ", z=" << pose.z
              << "], [roll=" << pose.roll << ", pitch=" << pose.pitch
              << ", yaw=" << pose.yaw <<"])";
    return outStream;
  }
};

struct TimestampedPose : Pose
{
  double timestamp;

  TimestampedPose()
  : Pose(),
    timestamp()
  {}

  TimestampedPose(const Pose& pose, const double timestamp)
  : Pose(pose),
    timestamp(timestamp)
  {}

  TimestampedPose& operator=(const TimestampedPose& other)
  {
    timestamp = other.timestamp;
    x = other.x;
    y = other.y;
    z = other.z;
    roll = other.roll;
    pitch = other.pitch;
    yaw = other.yaw;
    return *this;
  }
};

} // namespace common
#endif //POSE_H
