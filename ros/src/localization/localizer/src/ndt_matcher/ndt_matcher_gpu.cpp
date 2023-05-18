#include <ndt_matcher/ndt_matcher.h>

#ifdef CUDA_FOUND
NdtMatcherGPU::NdtMatcherGPU(const double transformationEpsilon,
  const double stepSize,
  const double resolution,
  const int maximumIterations,
  const double outlierRatio)
{
  mNdt.SetResolution(resolution);
  mNdt.SetStepSize(stepSize);
  mNdt.SetTransformationEpsilon(transformationEpsilon);
  mNdt.SetMaximumIterations(maximumIterations);
  mNdt.SetOutlierRatio(outlierRatio);
}

void NdtMatcherGPU::InputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr &points)
{
  mNdt.SetInputSource(points);
}

void NdtMatcherGPU::InputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr &points)
{
  mNdt.SetInputTarget(points);
}

void NdtMatcherGPU::AlignPoints(Eigen::Matrix4f &guessMatrix)
{
  mNdt.Align(guessMatrix);
}

int NdtMatcherGPU::GetFinalNumIteration()
{
  return mNdt.GetFinalNumIteration();
}

double NdtMatcherGPU::GetFitnessScoreWithValidPoints()
{
  return mNdt.GetFitnessScoreWithRadiusSearch();
}

double NdtMatcherGPU::GetFitnessScore()
{
  return mNdt.GetFitnessScore();
}

double NdtMatcherGPU::GetNdtMatchingScore()
{
  return mNdt.GetNdtMatchingScore();
}

double NdtMatcherGPU::GetNdtMatchingScoreWithValidPoints()
{
  return mNdt.GetNdtMatchingScoreWithValidPoints();
}

Eigen::Matrix4f NdtMatcherGPU::GetFinalTransformationMatrix()
{
  return mNdt.GetFinalTransformation();
}

pcl::PointCloud<pcl::PointXYZ> NdtMatcherGPU::GetValidPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ndtValidPointCloudPtr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndices::Ptr validPointIndices(new pcl::PointIndices);
  validPointIndices->indices = mNdt.GetValidPointsIndices();

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(sourcePoints);
  extract.setIndices(validPointIndices);
  extract.setNegative(false);
  extract.filter(*ndtValidPointCloudPtr);

  return *ndtValidPointCloudPtr;
}

pcl::PointCloud<pcl::PointXYZ> NdtMatcherGPU::GetInvalidPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ndtInvalidPointCloudPtr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndices::Ptr validPointIndices(new pcl::PointIndices);
  validPointIndices->indices = mNdt.GetValidPointsIndices();

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(sourcePoints);
  extract.setIndices(validPointIndices);
  extract.setNegative(true);
  extract.filter(*ndtInvalidPointCloudPtr);

  return *ndtInvalidPointCloudPtr;
}

bool NdtMatcherGPU::IsConverged()
{
  return mNdt.HasConverged();
}

void NdtMatcherGPU::DoSegmentation(
  pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPoints,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &nearestMapPoints,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &objectPoints,
  const double maxDistance)
{
  mNdt.SetInputNearestPointsDistanceSource(inputPoints);
  mNdt.NearestPointsDistance();

  const auto & minDistances = mNdt.GetMinDistances();

  for (unsigned int i = 0; i < minDistances.size(); ++i)
  {
    if (minDistances[i] > maxDistance)
    {
      objectPoints->points.push_back(inputPoints->points[i]);
    }
    else
    {
      nearestMapPoints->points.push_back(inputPoints->points[i]);
    }
  }
}
#endif //CUDA_FOUND
