#include <ndt_matcher/ndt_matcher.h>

NdtMatcherCPU::NdtMatcherCPU(const double transformationEpsilon,
  const double stepSize,
  const double resolution,
  const int maximumIterations,
  const double outlierRatio)
  : mOutputPoints(new pcl::PointCloud<pcl::PointXYZ>)
{
  mNdt.setTransformationEpsilon(transformationEpsilon);
  mNdt.setStepSize(stepSize);
  mNdt.setResolution(resolution);
  mNdt.setMaximumIterations(maximumIterations);
  mNdt.setOulierRatio(outlierRatio);
}

void NdtMatcherCPU::InputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr &points)
{
  mNdt.setInputSource(points);
}

void NdtMatcherCPU::InputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr &points)
{
  mNdt.setInputTarget(points);
}

void NdtMatcherCPU::AlignPoints(Eigen::Matrix4f &guessMatrix)
{
  mNdt.align(*mOutputPoints, guessMatrix);
}

int NdtMatcherCPU::GetFinalNumIteration()
{
  return mNdt.getFinalNumIteration();
}

double NdtMatcherCPU::GetFitnessScore()
{
  return mNdt.getFitnessScore();
}

double NdtMatcherCPU::GetFitnessScoreWithValidPoints()
{
  return mNdt.getFitnessScore();
}

double NdtMatcherCPU::GetNdtMatchingScore()
{
  return mNdt.getTransformationProbability();
}

double NdtMatcherCPU::GetNdtMatchingScoreWithValidPoints()
{
  return mNdt.getTransformationProbability();
}

Eigen::Matrix4f NdtMatcherCPU::GetFinalTransformationMatrix()
{
  return mNdt.getFinalTransformation();
}

pcl::PointCloud<pcl::PointXYZ> NdtMatcherCPU::GetValidPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints)
{
  ROS_WARN("GetValidPoints is not supported in CPU localization.");
  return pcl::PointCloud<pcl::PointXYZ>();
}

pcl::PointCloud<pcl::PointXYZ> NdtMatcherCPU::GetInvalidPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints)
{
  ROS_WARN("GetInvalidPoints is not supported in CPU localization.");
  return pcl::PointCloud<pcl::PointXYZ>();
}

bool NdtMatcherCPU::IsConverged()
{
  return mNdt.hasConverged();
}

void NdtMatcherCPU::DoSegmentation(
  pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPoints,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &nearestMapPoints,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &objectPoints,
  const double maxDistance)
{
  ROS_INFO("Use CPU localization now, "
    "this functionality only run in GPU localization.");
}
