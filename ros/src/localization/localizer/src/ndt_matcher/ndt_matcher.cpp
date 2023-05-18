#include <ndt_matcher/ndt_matcher.h>

Eigen::Matrix4f NdtMatcher::SetGuessMatrix(
  common::Pose &inputPose, Eigen::Matrix4f &transformMatrix)
{
  Eigen::Translation3f initTranslation(inputPose.x, inputPose.y, inputPose.z);
  Eigen::AngleAxisf initRotationX(inputPose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf initRotationY(inputPose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf initRotationZ(inputPose.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f outputMatrix =
    (initTranslation * initRotationZ * initRotationY * initRotationX) *
    transformMatrix;

  return outputMatrix;
}
