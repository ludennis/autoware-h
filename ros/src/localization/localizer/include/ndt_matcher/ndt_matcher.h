#ifndef ITRI_NDT_MATCHER_H
#define ITRI_NDT_MATCHER_H

#include <common/pose.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#ifdef CUDA_FOUND
#include <ndt_matcher/ndt_gpu/normal_distribution_transform.h>
#include <ndt_matcher/ndt_gpu/memory.h>
#endif //CUDA_FOUND

/*
 * The NdtMatcher aligns every rangefinder LiDAR scan, passed down from Localizer,
 * onto the current loaded map from PointMapLoader, to calculate a four-by-four
 * transformation matrix that maps the LiDAR scan onto the map. Furthermore,
 * such a matrix is used to publish ROS tf and determine the current pose of
 * vehicle in respect to the map’s coordinate system.
 */
class NdtMatcher
{
public:
  virtual void InputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr &points) = 0;
  virtual void InputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr &points) = 0;
  virtual void AlignPoints(Eigen::Matrix4f &guessMatrix) = 0;
  virtual int GetFinalNumIteration() = 0;
  virtual double GetFitnessScore() = 0;
  virtual double GetFitnessScoreWithValidPoints() = 0;
  virtual double GetNdtMatchingScore() = 0;
  virtual double GetNdtMatchingScoreWithValidPoints() = 0;
  virtual Eigen::Matrix4f GetFinalTransformationMatrix() = 0;
  virtual pcl::PointCloud<pcl::PointXYZ> GetValidPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints) = 0;
  virtual pcl::PointCloud<pcl::PointXYZ> GetInvalidPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints) = 0;
  virtual bool IsConverged() = 0;
  virtual void DoSegmentation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPoints,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &nearestMapPoints,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &objectPoints,
    const double maxDistance) = 0;
  static Eigen::Matrix4f SetGuessMatrix(common::Pose &inputPose, Eigen::Matrix4f &transformMatrix);

  virtual ~NdtMatcher() {}
};

class NdtMatcherCPU : public NdtMatcher
{
public:
  NdtMatcherCPU(const double transformationEpsilon = 0.01,
    const double stepSize = 0.1,
    const double resolution = 1.0,
    const int maximumIterations = 30,
    const double outlierRatio = 0.35);

  void InputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr &points) override;
  void InputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr &points) override;
  void AlignPoints(Eigen::Matrix4f &guessMatrix) override;
  int GetFinalNumIteration() override;
  double GetFitnessScore() override;
  double GetFitnessScoreWithValidPoints() override;
  double GetNdtMatchingScore() override;
  double GetNdtMatchingScoreWithValidPoints() override;
  Eigen::Matrix4f GetFinalTransformationMatrix() override;
  pcl::PointCloud<pcl::PointXYZ> GetValidPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints) override;
  pcl::PointCloud<pcl::PointXYZ> GetInvalidPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints) override;
  bool IsConverged() override;
  void DoSegmentation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPoints,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &nearestMapPoints,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &objectPoints,
    const double maxDistance) override;

private:
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> mNdt;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mOutputPoints;
};

#ifdef CUDA_FOUND
class NdtMatcherGPU: public NdtMatcher
{
public:
  NdtMatcherGPU(const double transformationEpsilon,
    const double stepSize,
    const double resolution,
    const int maximumIterations,
    const double outlierRatio);

  void InputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr &points) override;
  void InputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr &points) override;
  void AlignPoints(Eigen::Matrix4f &guessMatrix) override;
  int GetFinalNumIteration() override;
  double GetFitnessScore() override;
  double GetFitnessScoreWithValidPoints() override;
  double GetNdtMatchingScore() override;
  double GetNdtMatchingScoreWithValidPoints() override;
  Eigen::Matrix4f GetFinalTransformationMatrix() override;
  pcl::PointCloud<pcl::PointXYZ> GetValidPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints) override;
  pcl::PointCloud<pcl::PointXYZ> GetInvalidPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints) override;
  bool IsConverged() override;
  void DoSegmentation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inputPoints,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &nearestMapPoints,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &objectPoints,
    const double maxDistance) override;
private:
  gpu::GNormalDistributionsTransform mNdt;
};
#endif //CUDA_FOUND
#endif //ITRI_NDT_MATCHER_H
