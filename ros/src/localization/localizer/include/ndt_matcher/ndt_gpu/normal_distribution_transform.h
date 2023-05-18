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

#ifndef GPU_NDT_H_
#define GPU_NDT_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <eigen3/Eigen/Geometry>
#include <ndt_matcher/ndt_gpu/common.h>
#include <ndt_matcher/ndt_gpu/registration.h>
#include <ndt_matcher/ndt_gpu/voxel_grid.h>

namespace gpu
{

class GNormalDistributionsTransform: public GRegistration
{
public:
  GNormalDistributionsTransform();
  GNormalDistributionsTransform(const GNormalDistributionsTransform &other) = delete;
  void SetStepSize(double stepSize);
  void SetResolution(float resolution);
  void SetOutlierRatio(double olr);
  double GetStepSize() const;
  float GetResolution() const;
  double GetOutlierRatio() const;
  double GetNdtMatchingScore() const;
  double GetNdtMatchingScoreWithValidPoints() const;
  const std::vector<int> & GetValidPointsIndices() const;
  int GetRealIterations();
  const std::vector<double> & GetMinDistances() const;
  /* Set the input map points */
  void SetInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
  void SetInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr input);
  /* Compute and get fitness score */
  double GetFitnessScore(double maxRange = FLT_MAX);
  /* Compute and get fitness score with valid points from radius search */
  double GetFitnessScoreWithRadiusSearch(double maxRange = FLT_MAX);
  /* Get min distance of every point*/
  void NearestPointsDistance(double maxRange = FLT_MAX);

  ~GNormalDistributionsTransform();

protected:
  void ComputeTransformation(const Eigen::Matrix<float, 4, 4> &guess);
  double ComputeDerivatives(Eigen::Matrix<double, 6, 1> &scoreGradient,
    Eigen::Matrix<double, 6, 6> &hessian,
    std::shared_ptr<float> &transformedX, std::shared_ptr<float> &transformedY,
    std::shared_ptr<float> &transformedZ, int pointsNumber,
    Eigen::Matrix<double, 6, 1> pose, bool hasComputedHessian = true);

private:
  double AuxilaryFunctionPsiMT(double a,
    double fA, double f0, double g0, double mu = 1.e-4);
  double AuxilaryFunctionDPsiMT(double gA, double g0, double mu = 1.e-4);
  double UpdateIntervalMT (double &aL, double &fL, double &gL,
    double &aU, double &fU, double &gU,
    double aT, double fT, double gT);
  double TrialValueSelectionMT (double aL, double fL, double gL,
    double aU, double fU, double gU,
    double aT, double fT, double gT);
  void TransformPointCloud(std::shared_ptr<float> &inputX,
    std::shared_ptr<float> &inputY, std::shared_ptr<float> &inputZ,
    std::shared_ptr<float> &transformedX, std::shared_ptr<float> &transformedY,
    std::shared_ptr<float> &transformedZ, int pointsNumber,
    Eigen::Matrix<float, 4, 4> transform);
  void ComputeAngleDerivatives(MatrixHost pose, bool computeHessian = true);
  double ComputeStepLengthMT(const Eigen::Matrix<double, 6, 1> &x,
    Eigen::Matrix<double, 6, 1> &stepDir, double stepInit, double stepMax,
    double stepMin, double &score, Eigen::Matrix<double, 6, 1> &scoreGradient,
    Eigen::Matrix<double, 6, 6> &hessian,
    std::shared_ptr<float> &transformedX, std::shared_ptr<float> &transformedY,
    std::shared_ptr<float> &transformedZ, int pointsNumber);
  void ComputeHessian(Eigen::Matrix<double, 6, 6> &hessian,
    std::shared_ptr<float> &transformedX, std::shared_ptr<float> &transformedY,
    std::shared_ptr<float> &transformedZ, int pointsNumber,
    Eigen::Matrix<double, 6, 1> &p);

  double mGaussD1, mGaussD2;
  double mOutlierRatio;
  MatrixHost mJAngMatrix;
  MatrixHost mHAngMatrix;
  MatrixDevice mDJAngMatrix;
  MatrixDevice mDHAngMatrix;

  double mStepSize;
  float mResolution;
  double mTransProbability;
  double mTransProbabilityWithValidPoints;
  int mRealIterations;

  std::shared_ptr<int> mValidPointsIndices;
  std::shared_ptr<int> mValidVoxelNumberPerValidPoint;
  std::shared_ptr<int> mVoxelIndices;
  int mValidVoxelNumber;
  int mValidPointsNumber;

  GVoxelGrid mVoxelGrid;
};

} // namespace gpu

#endif // GPU_NDT_H_
