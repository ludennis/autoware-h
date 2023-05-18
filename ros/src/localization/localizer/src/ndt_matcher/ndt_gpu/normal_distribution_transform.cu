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

#include <ndt_matcher/ndt_gpu/debug.h>
#include <ndt_matcher/ndt_gpu/normal_distribution_transform.h>
#include <ndt_matcher/ndt_gpu/normal_distribution_transform_kernel.cuh>

#include <cmath>
#include <iostream>
#include <pcl/common/transforms.h>

namespace gpu
{

GNormalDistributionsTransform::GNormalDistributionsTransform()
{
  mGaussD1 = mGaussD2 = 0;
  mOutlierRatio = 0.55;
  mStepSize = 0.1;
  mResolution = 1.0f;
  mTransProbability = 0;

  double gaussC1, gaussC2, gaussD3;
  gaussC1 = 10.0 * (1 - mOutlierRatio);
  gaussC2 = mOutlierRatio / pow (mResolution, 3);
  gaussD3 = -log (gaussC2);
  mGaussD1 = -log (gaussC1 + gaussC2) - gaussD3;
  mGaussD2 = -2 * log((-log(gaussC1 * exp( -0.5 ) + gaussC2) - gaussD3) / mGaussD1);

  mTransformationEpsilon = 0.1;
  mMaxIterations = 35;

  mJAngMatrix = MatrixHost(24, 1);
  mHAngMatrix = MatrixHost(45, 1);
  mDJAngMatrix = MatrixDevice(24, 1);
  mDHAngMatrix = MatrixDevice(45, 1);

  mRealIterations = 0;
}

GNormalDistributionsTransform::~GNormalDistributionsTransform()
{
  mDJAngMatrix.MemFree();
  mDHAngMatrix.MemFree();
}

void GNormalDistributionsTransform::SetStepSize(double stepSize)
{
  mStepSize = stepSize;
}

void GNormalDistributionsTransform::SetResolution(float resolution)
{
  mResolution = resolution;
}

void GNormalDistributionsTransform::SetOutlierRatio(double olr)
{
  mOutlierRatio = olr;
}

double GNormalDistributionsTransform::GetStepSize() const
{
  return mStepSize;
}

float GNormalDistributionsTransform::GetResolution() const
{
  return mResolution;
}

double GNormalDistributionsTransform::GetOutlierRatio() const
{
  return mOutlierRatio;
}

double GNormalDistributionsTransform::GetNdtMatchingScore() const
{
  return mTransProbability;
}

double GNormalDistributionsTransform::GetNdtMatchingScoreWithValidPoints() const
{
  return mTransProbabilityWithValidPoints;
}
const std::vector<int> &
  GNormalDistributionsTransform::GetValidPointsIndices() const
{
  return mVoxelGrid.GetValidPointsIndices();
}

int GNormalDistributionsTransform::GetRealIterations()
{
  return mRealIterations;
}

const std::vector<double> & GNormalDistributionsTransform::GetMinDistances() const
{
  return mVoxelGrid.GetMinDistances();
}

double GNormalDistributionsTransform::AuxilaryFunctionPsiMT(double a,
  double fA, double f0, double g0, double mu)
{
  return (fA - f0 - mu * g0 * a);
}

double GNormalDistributionsTransform::AuxilaryFunctionDPsiMT(double gA,
  double g0, double mu)
{
  return (gA - mu * g0);
}

void GNormalDistributionsTransform::SetInputTarget(
  pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
  GRegistration::SetInputTarget(input);

  if (mTargetPointsNumber != 0)
  {
    mVoxelGrid.SetLeafSize(mResolution, mResolution, mResolution);
    mVoxelGrid.SetInput(mTargetPointsX,
      mTargetPointsY, mTargetPointsZ, mTargetPointsNumber);
  }
}

void GNormalDistributionsTransform::SetInputTarget(
  pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  GRegistration::SetInputTarget(input);

  if (mTargetPointsNumber != 0)
  {
    mVoxelGrid.SetLeafSize(mResolution, mResolution, mResolution);
    mVoxelGrid.SetInput(mTargetPointsX,
      mTargetPointsY, mTargetPointsZ, mTargetPointsNumber);
  }
}

void GNormalDistributionsTransform::ComputeTransformation(
  const Eigen::Matrix<float, 4, 4> &guess)
{
  if (mDJAngMatrix.IsEmpty())
  {
    mDJAngMatrix.MemAlloc();
  }

  if (mDHAngMatrix.IsEmpty())
  {
    mDHAngMatrix.MemAlloc();
  }

  mNumberIteration = 0;
  mConverged = false;

  double gaussC1, gaussC2, gaussD3;
  gaussC1 = 10 * ( 1 - mOutlierRatio);
  gaussC2 = mOutlierRatio / pow(mResolution, 3);
  gaussD3 = - log(gaussC2);
  mGaussD1 = -log(gaussC1 + gaussC2) - gaussD3;
  mGaussD2 = -2 * log((-log(gaussC1 * exp(-0.5) + gaussC2) - gaussD3) / mGaussD1);

  if (guess != Eigen::Matrix4f::Identity())
  {
    mFinalTransformationMatrix = guess;
    TransformPointCloud(mSourcePointsX, mSourcePointsY, mSourcePointsZ,
      mTransformedPointsX, mTransformedPointsY, mTransformedPointsZ,
      mSourcePointsNumber, guess);
  }

  Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> eigTransformation;
  eigTransformation.matrix() = mFinalTransformationMatrix;

  Eigen::Matrix<double, 6, 1> p, deltaP, scoreGradient;
  Eigen::Vector3f initTranslation = eigTransformation.translation();
  Eigen::Vector3f initRotation = eigTransformation.rotation().eulerAngles(0, 1, 2);

  p << initTranslation(0), initTranslation(1), initTranslation(2),
    initRotation(0), initRotation(1), initRotation(2);

  Eigen::Matrix<double, 6, 6> hessian;
  double score = 0;
  double deltaPNorm;
  score = ComputeDerivatives(scoreGradient,
    hessian, mTransformedPointsX, mTransformedPointsY, mTransformedPointsZ,
    mSourcePointsNumber, p);

  int loopTime = 0;
  while (!mConverged)
  {
    mPreviousTransformationMatrix = mTransformationMatrix;
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> sv(hessian,
      Eigen::ComputeFullU | Eigen::ComputeFullV);
    deltaP = sv.solve(-scoreGradient);
    deltaPNorm = deltaP.norm();

    if (deltaPNorm == 0 || deltaPNorm != deltaPNorm)
    {
      mTransProbability = score / static_cast<double>(mValidPointsNumber);
      mConverged = deltaPNorm == deltaPNorm;
      return;
    }

    deltaP.normalize();
    deltaPNorm = ComputeStepLengthMT(p, deltaP, deltaPNorm, mStepSize,
      mTransformationEpsilon / 2, score, scoreGradient, hessian,
      mTransformedPointsX, mTransformedPointsY,
      mTransformedPointsZ, mSourcePointsNumber);
    deltaP *= deltaPNorm;

    Eigen::Translation<float, 3> translation(static_cast<float>(deltaP(0)),
      static_cast<float>(deltaP(1)), static_cast<float>(deltaP(2)));
    Eigen::AngleAxis<float> tmp1(static_cast<float>(deltaP(3)), Eigen::Vector3f::UnitX());
    Eigen::AngleAxis<float> tmp2(static_cast<float>(deltaP(4)), Eigen::Vector3f::UnitY());
    Eigen::AngleAxis<float> tmp3(static_cast<float>(deltaP(5)), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxis<float> tmp4(tmp1 * tmp2 * tmp3);

    mTransformationMatrix = (translation * tmp4).matrix();
    p = p + deltaP;

    if (mNumberIteration > mMaxIterations || (mNumberIteration &&
      (std::fabs(deltaPNorm) < mTransformationEpsilon)))
    {
      mConverged = true;
    }

    mNumberIteration++;
    loopTime++;
  }
  mTransProbability = score / static_cast<double>(mSourcePointsNumber);
  mTransProbabilityWithValidPoints =
    score / static_cast<double>(mValidPointsNumber);
}

double GNormalDistributionsTransform::ComputeDerivatives(
  Eigen::Matrix<double, 6, 1> &scoreGradient,
  Eigen::Matrix<double, 6, 6> &hessian,
  std::shared_ptr<float> &transformedX, std::shared_ptr<float> &transformedY,
  std::shared_ptr<float> &transformedZ, int pointsNumber,
  Eigen::Matrix<double, 6, 1> pose, bool hasComputedHessian)
{
  MatrixHost p(6, 1);
  for (int i = 0; i < 6; i++)
  {
    p(i) = pose(i, 0);
  }

  scoreGradient.setZero();
  hessian.setZero();

  ComputeAngleDerivatives(p);

  mVoxelGrid.RadiusSearch(transformedX, transformedY, transformedZ,
    pointsNumber, mResolution, INT_MAX, mValidPointsIndices,
    mValidVoxelNumberPerValidPoint, mVoxelIndices, mValidVoxelNumber, mValidPointsNumber);

  std::shared_ptr<double> covariance = mVoxelGrid.GetCovarianceList();
  std::shared_ptr<double> inverseCovariance = mVoxelGrid.GetInverseCovarianceList();
  std::shared_ptr<double> centroid = mVoxelGrid.GetCentroidList();
  std::shared_ptr<int> pointsPerVoxel = mVoxelGrid.GetPointsPerVoxelList();
  int voxelNum = mVoxelGrid.GetVoxelNum();

  if (mValidPointsNumber == 0)
    return 0;

  std::shared_ptr<double> gradients =
    AllocateCudaMemory<double>(mValidPointsNumber * 6);
  std::shared_ptr<double> hessians =
    AllocateCudaMemory<double>(mValidPointsNumber * 6 * 6);
  std::shared_ptr<double> pointGradients =
    AllocateCudaMemory<double>(mValidPointsNumber * 3 * 6);
  std::shared_ptr<double> pointHessians =
    AllocateCudaMemory<double>(mValidPointsNumber * 18 * 6);
  std::shared_ptr<double> score =
    AllocateCudaMemory<double>(mValidPointsNumber);

  checkCudaErrors(cudaMemset(gradients.get(), 0,
    sizeof(double) * mValidPointsNumber * 6));
  checkCudaErrors(cudaMemset(hessians.get(), 0,
    sizeof(double) * mValidPointsNumber * 6 * 6));
  checkCudaErrors(cudaMemset(pointGradients.get(), 0,
    sizeof(double) * mValidPointsNumber * 3 * 6));
  checkCudaErrors(cudaMemset(pointHessians.get(), 0,
    sizeof(double) * mValidPointsNumber * 18 * 6));

  int blockX = (mValidPointsNumber > BLOCK_SIZE_X) ? BLOCK_SIZE_X : mValidPointsNumber;
  int gridX = (mValidPointsNumber - 1) / blockX + 1;
  dim3 grid;

  ComputePointGradients0<<<gridX, blockX>>>(mSourcePointsX.get(),
    mSourcePointsY.get(), mSourcePointsZ.get(),
    mValidPointsIndices.get(), mValidPointsNumber,
    mDJAngMatrix.buffer(),
    pointGradients.get(),
    pointGradients.get() + mValidPointsNumber * 7,
    pointGradients.get() + mValidPointsNumber * 14,
    pointGradients.get() + mValidPointsNumber * 9,
    pointGradients.get() + mValidPointsNumber * 15,
    pointGradients.get() + mValidPointsNumber * 4,
    pointGradients.get() + mValidPointsNumber * 10);
  checkCudaErrors(cudaGetLastError());

  ComputePointGradients1<<<gridX, blockX>>>(mSourcePointsX.get(),
    mSourcePointsY.get(), mSourcePointsZ.get(),
    mValidPointsIndices.get(), mValidPointsNumber,
    mDJAngMatrix.buffer(),
    pointGradients.get() + mValidPointsNumber * 16,
    pointGradients.get() + mValidPointsNumber * 5,
    pointGradients.get() + mValidPointsNumber * 11,
    pointGradients.get() + mValidPointsNumber * 17);
  checkCudaErrors(cudaGetLastError());

  if (hasComputedHessian)
  {
    ComputePointHessian0<<<gridX, blockX>>>(mSourcePointsX.get(),
      mSourcePointsY.get(), mSourcePointsZ.get(),
      mValidPointsIndices.get(), mValidPointsNumber,
      mDHAngMatrix.buffer(),
      pointHessians.get() + mValidPointsNumber * 57, pointHessians.get() +
        mValidPointsNumber * 63, pointHessians.get() + mValidPointsNumber * 69,
      pointHessians.get() + mValidPointsNumber * 75, pointHessians.get() +
        mValidPointsNumber * 58, pointHessians.get() + mValidPointsNumber * 81,
      pointHessians.get() + mValidPointsNumber * 64, pointHessians.get() +
        mValidPointsNumber * 87, pointHessians.get() + mValidPointsNumber * 70,
      pointHessians.get() + mValidPointsNumber * 93, pointHessians.get() +
        mValidPointsNumber * 59, pointHessians.get() + mValidPointsNumber * 99,
      pointHessians.get() + mValidPointsNumber * 65, pointHessians.get() +
        mValidPointsNumber * 105, pointHessians.get() + mValidPointsNumber * 71);

    checkCudaErrors(cudaGetLastError());

    ComputePointHessian1<<<gridX, blockX>>>(mSourcePointsX.get(),
      mSourcePointsY.get(), mSourcePointsZ.get(),
      mValidPointsIndices.get(), mValidPointsNumber,
      mDHAngMatrix.buffer(),
      pointHessians.get() + mValidPointsNumber * 76, pointHessians.get() +
        mValidPointsNumber * 82, pointHessians.get() + mValidPointsNumber * 88,
      pointHessians.get() + mValidPointsNumber * 94, pointHessians.get() +
        mValidPointsNumber * 77, pointHessians.get() + mValidPointsNumber * 100,
      pointHessians.get() + mValidPointsNumber * 83, pointHessians.get() +
        mValidPointsNumber * 106, pointHessians.get() + mValidPointsNumber * 89);
    checkCudaErrors(cudaGetLastError());

    ComputePointHessian2<<<gridX, blockX>>>(mSourcePointsX.get(),
      mSourcePointsY.get(), mSourcePointsZ.get(),
      mValidPointsIndices.get(), mValidPointsNumber,
      mDHAngMatrix.buffer(),
      pointHessians.get() + mValidPointsNumber * 95, pointHessians.get() +
        mValidPointsNumber * 101, pointHessians.get() + mValidPointsNumber * 107);
    checkCudaErrors(cudaGetLastError());
  }
  checkCudaErrors(cudaDeviceSynchronize());

  std::shared_ptr<double> tmpHessian =
    AllocateCudaMemory<double>(mValidVoxelNumber * 6);
  std::shared_ptr<double> exCovX =
    AllocateCudaMemory<double>(mValidVoxelNumber);
  std::shared_ptr<double> covDxdPi =
    AllocateCudaMemory<double>(mValidVoxelNumber * 3 * 6);

  ComputeExCovX<<<gridX, blockX>>>(transformedX.get(), transformedY.get(),
    transformedZ.get(), mValidPointsIndices.get(),
    mValidVoxelNumberPerValidPoint.get(),
    mVoxelIndices.get(), mValidPointsNumber, centroid.get(), centroid.get() + voxelNum,
    centroid.get() + 2 * voxelNum, mGaussD1, mGaussD2,
    exCovX.get(), inverseCovariance.get(), inverseCovariance.get() + voxelNum,
    inverseCovariance.get() + 2 * voxelNum,
    inverseCovariance.get() + 3 * voxelNum,
    inverseCovariance.get() + 4 * voxelNum,
    inverseCovariance.get() + 5 * voxelNum,
    inverseCovariance.get() + 6 * voxelNum,
    inverseCovariance.get() + 7 * voxelNum,
    inverseCovariance.get() + 8 * voxelNum);
  checkCudaErrors(cudaGetLastError());

  ComputeScoreList<<<gridX, blockX>>>(mValidVoxelNumberPerValidPoint.get(),
    mValidPointsNumber, exCovX.get(), mGaussD1, score.get());
  checkCudaErrors(cudaGetLastError());

  int block_x2 = (mValidVoxelNumber > BLOCK_SIZE_X) ? BLOCK_SIZE_X : mValidVoxelNumber;
  int grid_x2 = (mValidVoxelNumber - 1) / block_x2 + 1;
  UpdateExCovX<<<grid_x2, block_x2>>>(exCovX.get(), mGaussD2, mValidVoxelNumber);
  checkCudaErrors(cudaGetLastError());

  grid.x = gridX;
  grid.y = 3;
  grid.z = 6;
  ComputeCovDxdPi<<<grid, blockX>>>(
    mValidVoxelNumberPerValidPoint.get(), mVoxelIndices.get(),
    mValidPointsNumber, inverseCovariance.get(), voxelNum,
    pointGradients.get(), covDxdPi.get(), mValidVoxelNumber);
  checkCudaErrors(cudaGetLastError());

  grid.x = gridX;
  grid.y = 6;
  grid.z = 1;
  ComputeScoreGradientList<<<grid, blockX>>>(transformedX.get(),
    transformedY.get(), transformedZ.get(), mValidPointsIndices.get(),
    mValidVoxelNumberPerValidPoint.get(), mVoxelIndices.get(), mValidPointsNumber,
    centroid.get(), centroid.get() + voxelNum, centroid.get() + 2 * voxelNum,
    exCovX.get(), covDxdPi.get(), mGaussD1, mValidVoxelNumber, gradients.get());
  checkCudaErrors(cudaGetLastError());

  if (hasComputedHessian)
  {
    grid.y = 6;
    grid.z = 1;
    ComputeHessianListS0<<<grid, blockX>>>(transformedX.get(),
      transformedY.get(), transformedZ.get(), mValidPointsIndices.get(),
      mValidVoxelNumberPerValidPoint.get(), mVoxelIndices.get(), mValidPointsNumber,
      centroid.get(), centroid.get() + voxelNum, centroid.get() + 2 * voxelNum,
      inverseCovariance.get(), inverseCovariance.get() + voxelNum,
      inverseCovariance.get() + 2 * voxelNum,
      inverseCovariance.get() + 3 * voxelNum,
      inverseCovariance.get() + 4 * voxelNum,
      inverseCovariance.get() + 5 * voxelNum,
      inverseCovariance.get() + 6 * voxelNum,
      inverseCovariance.get() + 7 * voxelNum,
      inverseCovariance.get() + 8 * voxelNum,
      pointGradients.get(), tmpHessian.get(), mValidVoxelNumber);
    checkCudaErrors(cudaGetLastError());

    grid.z = 6;
    ComputeHessianListS1<<<grid, blockX>>>(transformedX.get(),
      transformedY.get(), transformedZ.get(), mValidPointsIndices.get(),
      mValidVoxelNumberPerValidPoint.get(), mVoxelIndices.get(), mValidPointsNumber,
      centroid.get(), centroid.get() + voxelNum, centroid.get() + 2 * voxelNum,
      mGaussD1, mGaussD2, hessians.get(),
      exCovX.get(), tmpHessian.get(), covDxdPi.get(),
      pointGradients.get(), mValidVoxelNumber);
    checkCudaErrors(cudaGetLastError());

    ComputeHessianListS2<<<grid, blockX>>>(transformedX.get(),
      transformedY.get(), transformedZ.get(), mValidPointsIndices.get(),
      mValidVoxelNumberPerValidPoint.get(), mVoxelIndices.get(), mValidPointsNumber,
      centroid.get(), centroid.get() + voxelNum, centroid.get() + 2 * voxelNum,
      mGaussD1, exCovX.get(), inverseCovariance.get(),
      inverseCovariance.get() + voxelNum,
      inverseCovariance.get() + 2 * voxelNum,
      inverseCovariance.get() + 3 * voxelNum,
      inverseCovariance.get() + 4 * voxelNum,
      inverseCovariance.get() + 5 * voxelNum,
      inverseCovariance.get() + 6 * voxelNum,
      inverseCovariance.get() + 7 * voxelNum,
      inverseCovariance.get() + 8 * voxelNum,
      pointHessians.get(), hessians.get());
    checkCudaErrors(cudaGetLastError());
  }

  int fullSize = mValidPointsNumber;
  int halfSize = (fullSize - 1) / 2 + 1;

  while (fullSize > 1)
  {
    blockX = (halfSize > BLOCK_SIZE_X) ? BLOCK_SIZE_X : halfSize;
    gridX = (halfSize - 1) / blockX + 1;
    grid.x = gridX;
    grid.y = 1;
    grid.z = 6;
    MatrixSum<<<grid, blockX>>>(gradients.get(),
      fullSize, halfSize, 1, 6, mValidPointsNumber);
    checkCudaErrors(cudaGetLastError());

    grid.y = 6;
    MatrixSum<<<grid, blockX>>>(hessians.get(),
      fullSize, halfSize, 6, 6, mValidPointsNumber);
    checkCudaErrors(cudaGetLastError());

    SumScore<<<gridX, blockX>>>(score.get(), fullSize, halfSize);
    checkCudaErrors(cudaGetLastError());

    fullSize = halfSize;
    halfSize = (fullSize - 1) / 2 + 1;
  }
  checkCudaErrors(cudaDeviceSynchronize());

  MatrixDevice dgrad(1, 6, mValidPointsNumber, gradients.get()),
    dhess(6, 6, mValidPointsNumber, hessians.get());
  MatrixHost hgrad(1, 6), hhess(6, 6);

  hgrad.MoveToHost(dgrad);
  hhess.MoveToHost(dhess);

  for (int i = 0; i < 6; i++)
  {
    scoreGradient(i) = hgrad(i);
  }

  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      hessian(i, j) = hhess(i, j);
    }
  }

  double scoreInc;
  checkCudaErrors(cudaMemcpy(&scoreInc,
    score.get(), sizeof(double), cudaMemcpyDeviceToHost));

  return scoreInc;
}

void GNormalDistributionsTransform::ComputeAngleDerivatives(MatrixHost pose,
  bool computeHessian)
{
  double cx, cy, cz, sx, sy, sz;
  if (fabs(pose(3)) < 10e-5)
  {
    cx = 1.0;
    sx = 0.0;
  }
  else
  {
    cx = cos(pose(3));
    sx = sin(pose(3));
  }

  if (fabs(pose(4)) < 10e-5)
  {
    cy = 1.0;
    sy = 0.0;
  }
  else
  {
    cy = cos(pose(4));
    sy = sin(pose(4));
  }

  if (fabs(pose(5)) < 10e-5)
  {
    cz = 1.0;
    sz = 0.0;
  }
  else
  {
    cz = cos(pose(5));
    sz = sin(pose(5));
  }

  mJAngMatrix(0) = -sx * sz + cx * sy * cz;
  mJAngMatrix(1) = -sx * cz - cx * sy * sz;
  mJAngMatrix(2) = -cx * cy;

  mJAngMatrix(3) = cx * sz + sx * sy * cz;
  mJAngMatrix(4) = cx * cz - sx * sy * sz;
  mJAngMatrix(5) = -sx * cy;

  mJAngMatrix(6) = -sy * cz;
  mJAngMatrix(7) = sy * sz;
  mJAngMatrix(8) = cy;

  mJAngMatrix(9) = sx * cy * cz;
  mJAngMatrix(10) = -sx * cy * sz;
  mJAngMatrix(11) = sx * sy;

  mJAngMatrix(12) = -cx * cy * cz;
  mJAngMatrix(13) = cx * cy * sz;
  mJAngMatrix(14) = -cx * sy;

  mJAngMatrix(15) = -cy * sz;
  mJAngMatrix(16) = -cy * cz;
  mJAngMatrix(17) = 0;

  mJAngMatrix(18) = cx * cz - sx * sy * sz;
  mJAngMatrix(19) = -cx * sz - sx * sy * cz;
  mJAngMatrix(20) = 0;

  mJAngMatrix(21) = sx * cz + cx * sy * sz;
  mJAngMatrix(22) = cx * sy * cz - sx * sz;
  mJAngMatrix(23) = 0;

  mJAngMatrix.MoveToGpu(mDJAngMatrix);

  if (computeHessian)
  {
    mHAngMatrix(0) = -cx * sz - sx * sy * cz;
    mHAngMatrix(1) = -cx * cz + sx * sy * sz;
    mHAngMatrix(2) = sx * cy;

    mHAngMatrix(3) = -sx * sz + cx * sy * cz;
    mHAngMatrix(4) = -cx * sy * sz - sx * cz;
    mHAngMatrix(5) = -cx * cy;

    mHAngMatrix(6) = cx * cy * cz;
    mHAngMatrix(7) = -cx * cy * sz;
    mHAngMatrix(8) = cx * sy;

    mHAngMatrix(9) = sx * cy * cz;
    mHAngMatrix(10) = -sx * cy * sz;
    mHAngMatrix(11) = sx * sy;

    mHAngMatrix(12) = -sx * cz - cx * sy * sz;
    mHAngMatrix(13) = sx * sz - cx * sy * cz;
    mHAngMatrix(14) = 0;

    mHAngMatrix(15) = cx * cz - sx * sy * sz;
    mHAngMatrix(16) = -sx * sy * cz - cx * sz;
    mHAngMatrix(17) = 0;

    mHAngMatrix(18) = -cy * cz;
    mHAngMatrix(19) = cy * sz;
    mHAngMatrix(20) = sy;

    mHAngMatrix(21) = -sx * sy * cz;
    mHAngMatrix(22) = sx * sy * sz;
    mHAngMatrix(23) = sx * cy;

    mHAngMatrix(24) = cx * sy * cz;
    mHAngMatrix(25) = -cx * sy * sz;
    mHAngMatrix(26) = -cx * cy;

    mHAngMatrix(27) = sy * sz;
    mHAngMatrix(28) = sy * cz;
    mHAngMatrix(29) = 0;

    mHAngMatrix(30) = -sx * cy * sz;
    mHAngMatrix(31) = -sx * cy * cz;
    mHAngMatrix(32) = 0;

    mHAngMatrix(33) = cx * cy * sz;
    mHAngMatrix(34) = cx * cy * cz;
    mHAngMatrix(35) = 0;

    mHAngMatrix(36) = -cy * cz;
    mHAngMatrix(37) = cy * sz;
    mHAngMatrix(38) = 0;

    mHAngMatrix(39) = -cx * sz - sx * sy * cz;
    mHAngMatrix(40) = -cx * cz + sx * sy * sz;
    mHAngMatrix(41) = 0;

    mHAngMatrix(42) = -sx * sz + cx * sy * cz;
    mHAngMatrix(43) = -cx * sy * sz - sx * cz;
    mHAngMatrix(44) = 0;

    mHAngMatrix.MoveToGpu(mDHAngMatrix);
  }
}

void GNormalDistributionsTransform::TransformPointCloud(std::shared_ptr<float> &inputX,
  std::shared_ptr<float> &inputY, std::shared_ptr<float> &inputZ,
  std::shared_ptr<float> &transformedX, std::shared_ptr<float> &transformedY,
  std::shared_ptr<float> &transformedZ, int pointsNumber,
  Eigen::Matrix<float, 4, 4> transform)
{
  Eigen::Transform<float, 3, Eigen::Affine> t(transform);
  MatrixHost htrans(3, 4);
  MatrixDevice dtrans(3, 4);
  dtrans.MemAlloc();

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      htrans(i, j) = t(i, j);
    }
  }

  htrans.MoveToGpu(dtrans);

  if (pointsNumber > 0)
  {
    int blockX = (pointsNumber <= BLOCK_SIZE_X) ? pointsNumber : BLOCK_SIZE_X;
    int gridX = (pointsNumber - 1) / blockX + 1;

    TransformUseGpu<<<gridX, blockX >>>(inputX.get(), inputY.get(), inputZ.get(),
      transformedX.get(), transformedY.get(), transformedZ.get(),
      pointsNumber, dtrans);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
  }
  dtrans.MemFree();
}

double GNormalDistributionsTransform::ComputeStepLengthMT(
  const Eigen::Matrix<double, 6, 1> &x,
  Eigen::Matrix<double, 6, 1> &stepDir, double stepInit, double stepMax,
  double stepMin, double &score, Eigen::Matrix<double, 6, 1> &scoreGradient,
  Eigen::Matrix<double, 6, 6> &hessian,
  std::shared_ptr<float> &transformedX, std::shared_ptr<float> &transformedY,
  std::shared_ptr<float> &transformedZ, int pointsNumber)
{
  double phi0 = -score;
  double dPhi0 = -(scoreGradient.dot(stepDir));
  Eigen::Matrix<double, 6, 1> xT;

  if (dPhi0 >= 0)
  {
    if (dPhi0 == 0)
      return 0;
    else
    {
      dPhi0 *= -1;
      stepDir *= -1;
    }
  }

  int maxStepIterations = 10;
  int stepIterations = 0;
  double mu = 1.e-4;
  double nu = 0.9;
  double aL = 0, aU = 0;
  double fL = AuxilaryFunctionPsiMT(aL, phi0, phi0, dPhi0, mu);
  double gL = AuxilaryFunctionDPsiMT(dPhi0, dPhi0, mu);
  double fU = AuxilaryFunctionPsiMT(aU, phi0, phi0, dPhi0, mu);
  double gU = AuxilaryFunctionDPsiMT(dPhi0, dPhi0, mu);
  bool intervalConverged = (stepMax - stepMin) > 0, openInterval = true;
  double aT = stepInit;
  aT = std::min(aT, stepMax);
  aT = std::max(aT, stepMin);
  xT = x + stepDir * aT;

  Eigen::Translation<float, 3> translation(static_cast<float>(xT(0)),
    static_cast<float>(xT(1)), static_cast<float>(xT(2)));
  Eigen::AngleAxis<float> tmp1(static_cast<float>(xT(3)), Eigen::Vector3f::UnitX());
  Eigen::AngleAxis<float> tmp2(static_cast<float>(xT(4)), Eigen::Vector3f::UnitY());
  Eigen::AngleAxis<float> tmp3(static_cast<float>(xT(5)), Eigen::Vector3f::UnitZ());
  Eigen::AngleAxis<float> tmp4(tmp1 * tmp2 * tmp3);

  mFinalTransformationMatrix = (translation * tmp4).matrix();
  TransformPointCloud(mSourcePointsX, mSourcePointsY, mSourcePointsZ,
    transformedX, transformedY, transformedZ,
    pointsNumber, mFinalTransformationMatrix);

  score = ComputeDerivatives(scoreGradient,
    hessian, transformedX, transformedY, transformedZ, pointsNumber, xT);

  double phiT = -score;
  double dPhiT = -(scoreGradient.dot(stepDir));
  double psiT = AuxilaryFunctionPsiMT(aT, phiT, phi0, dPhi0, mu);
  double dPsiT = AuxilaryFunctionDPsiMT(dPhiT, dPhi0, mu);

  while (!intervalConverged && stepIterations < maxStepIterations &&
    !(psiT <= 0 && dPhiT <= -nu * dPhi0))
  {
    if (openInterval)
    {
      aT = TrialValueSelectionMT(aL, fL, gL, aU, fU, gU, aT, psiT, dPsiT);
    }
    else
    {
      aT = TrialValueSelectionMT(aL, fL, gL, aU, fU, gU, aT, phiT, dPhiT);
    }

    aT = (aT < stepMax) ? aT : stepMax;
    aT = (aT > stepMin) ? aT : stepMin;
    xT = x + stepDir * aT;

    translation = Eigen::Translation<float, 3>(static_cast<float>(xT(0)),
      static_cast<float>(xT(1)), static_cast<float>(xT(2)));
    tmp1 = Eigen::AngleAxis<float>(static_cast<float>(xT(3)), Eigen::Vector3f::UnitX());
    tmp2 = Eigen::AngleAxis<float>(static_cast<float>(xT(4)), Eigen::Vector3f::UnitY());
    tmp3 = Eigen::AngleAxis<float>(static_cast<float>(xT(5)), Eigen::Vector3f::UnitZ());
    tmp4 = tmp1 * tmp2 * tmp3;

    mFinalTransformationMatrix = (translation * tmp4).matrix();
    TransformPointCloud(mSourcePointsX, mSourcePointsY, mSourcePointsZ,
      transformedX, transformedY, transformedZ,
      pointsNumber, mFinalTransformationMatrix);

    score = ComputeDerivatives(scoreGradient, hessian,
      transformedX, transformedY, transformedZ, pointsNumber, xT, false);

    phiT -= score;
    dPhiT -= (scoreGradient.dot(stepDir));
    psiT = AuxilaryFunctionPsiMT(aT, phiT, phi0, dPhi0, mu);
    dPsiT = AuxilaryFunctionDPsiMT(dPhiT, dPhi0, mu);

    if (openInterval && (psiT <= 0 && dPsiT >= 0))
    {
      openInterval = false;
      fL += phi0 - mu * dPhi0 * aL;
      gL += mu * dPhi0;
      fU += phi0 - mu * dPhi0 * aU;
      gU += mu * dPhi0;
    }

    if (openInterval)
    {
      intervalConverged =
        UpdateIntervalMT(aL, fL, gL, aU, fU, gU, aT, psiT, dPsiT);
    }
    else
    {
      intervalConverged =
        UpdateIntervalMT(aL, fL, gL, aU, fU, gU, aT, phiT, dPhiT);
    }
    stepIterations++;
  }

  if (stepIterations)
  {
    ComputeHessian(hessian, transformedX, transformedY, transformedZ,
      pointsNumber, xT);
  }

  mRealIterations += stepIterations;

  return aT;
}

double GNormalDistributionsTransform::TrialValueSelectionMT(
  double aL, double fL, double gL,
  double aU, double fU, double gU,
  double aT, double fT, double gT)
{
  if (fT > fL)
  {
    double z = 3 * (fT - fL) / (aT - aL) - gT - gL;
    double w = std::sqrt (z * z - gT * gL);
    double aC = aL + (aT - aL) * (w - gL - z) / (gT - gL + 2 * w);

    double aQ = aL - 0.5 * (aL - aT) * gL / (gL - (fL - fT) / (aL - aT));

    if (std::fabs (aC - aL) < std::fabs (aQ - aL))
      return (aC);
    else
      return (0.5 * (aQ + aC));
  }
  else if (gT * gL < 0)
  {
    double z = 3 * (fT - fL) / (aT - aL) - gT - gL;
    double w = std::sqrt (z * z - gT * gL);
    double aC = aL + (aT - aL) * (w - gL - z) / (gT - gL + 2 * w);

    double aS = aL - (aL - aT) / (gL - gT) * gL;

    if (std::fabs (aC - aT) >= std::fabs (aS - aT))
      return (aC);
    else
      return (aS);
  }
  else if (std::fabs (gT) <= std::fabs (gL))
  {
    double z = 3 * (fT - fL) / (aT - aL) - gT - gL;
    double w = std::sqrt (z * z - gT * gL);
    double aC = aL + (aT - aL) * (w - gL - z) / (gT - gL + 2 * w);

    double aS = aL - (aL - aT) / (gL - gT) * gL;

    double aTNext;

    if (std::fabs (aC - aT) < std::fabs (aS - aT))
      aTNext = aC;
    else
      aTNext = aS;

    if (aT > aL)
      return (std::min (aT + 0.66 * (aU - aT), aTNext));
    else
      return (std::max (aT + 0.66 * (aU - aT), aTNext));
  }
  else
  {
    double z = 3 * (fT - fU) / (aT - aU) - gT - gU;
    double w = std::sqrt (z * z - gT * gU);
    return (aU + (aT - aU) * (w - gU - z) / (gT - gU + 2 * w));
  }
}

double GNormalDistributionsTransform::UpdateIntervalMT(
  double &aL, double &fL, double &gL,
  double &aU, double &fU, double &gU,
  double aT, double fT, double gT)
{
  if (fT > fL)
  {
    aU = aT;
    fU = fT;
    gU = gT;
    return (false);
  }
  else if (gT * (aL - aT) > 0)
  {
    aL = aT;
    fL = fT;
    gL = gT;
    return (false);
  }
  else if (gT * (aL - aT) < 0)
  {
    aU = aL;
    fU = fL;
    gU = gL;

    aL = aT;
    fL = fT;
    gL = gT;
    return (false);
  }
  else
    return (true);
}

void GNormalDistributionsTransform::ComputeHessian(
  Eigen::Matrix<double, 6, 6> &hessian,
  std::shared_ptr<float> &transformedX, std::shared_ptr<float> &transformedY,
  std::shared_ptr<float> &transformedZ, int pointsNumber,
  Eigen::Matrix<double, 6, 1> &p)
{
  std::shared_ptr<double> centroid = mVoxelGrid.GetCentroidList();
  std::shared_ptr<double> covariance = mVoxelGrid.GetCovarianceList();
  std::shared_ptr<double> inverseCovariance = mVoxelGrid.GetInverseCovarianceList();
  std::shared_ptr<int> pointsPerVoxel = mVoxelGrid.GetPointsPerVoxelList();
  int voxelNum = mVoxelGrid.GetVoxelNum();

  if (mValidPointsNumber <= 0)
    return;

  std::shared_ptr<double> hessians
    = AllocateCudaMemory<double>(mValidPointsNumber * 6 * 6);
  std::shared_ptr<double> pointGradients
    = AllocateCudaMemory<double>(mValidPointsNumber * 3 * 6);
  std::shared_ptr<double> pointHessians
    = AllocateCudaMemory<double>(mValidPointsNumber * 18 * 6);

  checkCudaErrors(cudaMemset(hessians.get(), 0,
    sizeof(double) * mValidPointsNumber * 6 * 6));
  checkCudaErrors(cudaMemset(pointGradients.get(), 0,
    sizeof(double) * mValidPointsNumber * 3 * 6));
  checkCudaErrors(cudaMemset(pointHessians.get(), 0,
    sizeof(double) * mValidPointsNumber * 18 * 6));

  int blockX = (mValidPointsNumber > BLOCK_SIZE_X) ?
    BLOCK_SIZE_X : mValidPointsNumber;
  int gridX = (mValidPointsNumber - 1) / blockX + 1;
  dim3 grid;

  ComputePointGradients0<<<gridX, blockX>>>(mSourcePointsX.get(),
    mSourcePointsY.get(), mSourcePointsZ.get(),
    mValidPointsIndices.get(), mValidPointsNumber,
    mDJAngMatrix.buffer(),
    pointGradients.get(),
    pointGradients.get() + mValidPointsNumber * 7,
    pointGradients.get() + mValidPointsNumber * 14,
    pointGradients.get() + mValidPointsNumber * 9,
    pointGradients.get() + mValidPointsNumber * 15,
    pointGradients.get() + mValidPointsNumber * 4,
    pointGradients.get() + mValidPointsNumber * 10);
  checkCudaErrors(cudaGetLastError());

  ComputePointGradients1<<<gridX, blockX>>>(mSourcePointsX.get(),
    mSourcePointsY.get(), mSourcePointsZ.get(),
    mValidPointsIndices.get(), mValidPointsNumber,
    mDJAngMatrix.buffer(),
    pointGradients.get() + mValidPointsNumber * 16,
    pointGradients.get() + mValidPointsNumber * 5,
    pointGradients.get() + mValidPointsNumber * 11,
    pointGradients.get() + mValidPointsNumber * 17);
  checkCudaErrors(cudaGetLastError());

  ComputePointHessian0<<<gridX, blockX>>>(mSourcePointsX.get(),
    mSourcePointsY.get(), mSourcePointsZ.get(),
    mValidPointsIndices.get(), mValidPointsNumber,
    mDHAngMatrix.buffer(),
    pointHessians.get() + mValidPointsNumber * 57,
    pointHessians.get() + mValidPointsNumber * 63,
    pointHessians.get() + mValidPointsNumber * 69,
    pointHessians.get() + mValidPointsNumber * 75,
    pointHessians.get() + mValidPointsNumber * 58,
    pointHessians.get() + mValidPointsNumber * 81,
    pointHessians.get() + mValidPointsNumber * 64,
    pointHessians.get() + mValidPointsNumber * 87,
    pointHessians.get() + mValidPointsNumber * 70,
    pointHessians.get() + mValidPointsNumber * 93,
    pointHessians.get() + mValidPointsNumber * 59,
    pointHessians.get() + mValidPointsNumber * 99,
    pointHessians.get() + mValidPointsNumber * 65,
    pointHessians.get() + mValidPointsNumber * 105,
    pointHessians.get() + mValidPointsNumber * 71);
  checkCudaErrors(cudaGetLastError());

  ComputePointHessian1<<<gridX, blockX>>>(mSourcePointsX.get(),
    mSourcePointsY.get(), mSourcePointsZ.get(),
    mValidPointsIndices.get(), mValidPointsNumber,
    mDHAngMatrix.buffer(),
    pointHessians.get() + mValidPointsNumber * 76,
    pointHessians.get() + mValidPointsNumber * 82,
    pointHessians.get() + mValidPointsNumber * 88,
    pointHessians.get() + mValidPointsNumber * 94,
    pointHessians.get() + mValidPointsNumber * 77,
    pointHessians.get() + mValidPointsNumber * 100,
    pointHessians.get() + mValidPointsNumber * 83,
    pointHessians.get() + mValidPointsNumber * 106,
    pointHessians.get() + mValidPointsNumber * 89);
  checkCudaErrors(cudaGetLastError());

  ComputePointHessian2<<<gridX, blockX>>>(mSourcePointsX.get(),
    mSourcePointsY.get(), mSourcePointsZ.get(),
    mValidPointsIndices.get(), mValidPointsNumber,
    mDHAngMatrix.buffer(),
    pointHessians.get() + mValidPointsNumber * 95,
    pointHessians.get() + mValidPointsNumber * 101,
    pointHessians.get() + mValidPointsNumber * 107);
  checkCudaErrors(cudaGetLastError());

  std::shared_ptr<double> tmpHessian
    = AllocateCudaMemory<double>(mValidVoxelNumber * 6);
  std::shared_ptr<double> exCovX
    = AllocateCudaMemory<double>(mValidVoxelNumber);
  std::shared_ptr<double> covDxdPi
    = AllocateCudaMemory<double>(mValidVoxelNumber * 3 * 6);

  ComputeExCovX<<<gridX, blockX>>>(transformedX.get(),
    transformedY.get(), transformedZ.get(), mValidPointsIndices.get(),
    mValidVoxelNumberPerValidPoint.get(), mVoxelIndices.get(), mValidPointsNumber,
    centroid.get(), centroid.get() + voxelNum, centroid.get() + 2 * voxelNum,
    mGaussD1, mGaussD2,
    exCovX.get(), inverseCovariance.get(),
    inverseCovariance.get() + voxelNum,
    inverseCovariance.get() + 2 * voxelNum,
    inverseCovariance.get() + 3 * voxelNum,
    inverseCovariance.get() + 4 * voxelNum,
    inverseCovariance.get() + 5 * voxelNum,
    inverseCovariance.get() + 6 * voxelNum,
    inverseCovariance.get() + 7 * voxelNum,
    inverseCovariance.get() + 8 * voxelNum);

  checkCudaErrors(cudaGetLastError());

  grid.x = gridX;
  grid.y = 3;
  grid.z = 6;
  ComputeCovDxdPi<<<grid, blockX>>>(
    mValidVoxelNumberPerValidPoint.get(), mVoxelIndices.get(),
    mValidPointsNumber, inverseCovariance.get(), voxelNum,
    pointGradients.get(),
    covDxdPi.get(), mValidVoxelNumber);
  checkCudaErrors(cudaGetLastError());

  int block_x2 = (mValidVoxelNumber > BLOCK_SIZE_X) ?
    BLOCK_SIZE_X : mValidVoxelNumber;
  int grid_x2 = (mValidVoxelNumber - 1) / block_x2 + 1;

  UpdateExCovX<<<grid_x2, block_x2>>>(exCovX.get(), mGaussD2, mValidVoxelNumber);
  checkCudaErrors(cudaGetLastError());

  grid.y = 6;
  grid.z = 1;

  ComputeHessianListS0<<<grid, blockX>>>(transformedX.get(),
    transformedY.get(), transformedZ.get(), mValidPointsIndices.get(),
    mValidVoxelNumberPerValidPoint.get(), mVoxelIndices.get(), mValidPointsNumber,
    centroid.get(), centroid.get() + voxelNum, centroid.get() + 2 * voxelNum,
    inverseCovariance.get(), inverseCovariance.get() + voxelNum,
    inverseCovariance.get() + 2 * voxelNum,
    inverseCovariance.get() + 3 * voxelNum,
    inverseCovariance.get() + 4 * voxelNum,
    inverseCovariance.get() + 5 * voxelNum,
    inverseCovariance.get() + 6 * voxelNum,
    inverseCovariance.get() + 7 * voxelNum,
    inverseCovariance.get() + 8 * voxelNum,
    pointGradients.get(),
    tmpHessian.get(), mValidVoxelNumber);
  checkCudaErrors(cudaGetLastError());

  grid.z = 6;

  ComputeHessianListS1<<<grid, blockX>>>(transformedX.get(),
    transformedY.get(), transformedZ.get(), mValidPointsIndices.get(),
    mValidVoxelNumberPerValidPoint.get(), mVoxelIndices.get(), mValidPointsNumber,
    centroid.get(), centroid.get() + voxelNum, centroid.get() + 2 * voxelNum,
    mGaussD1, mGaussD2, hessians.get(),
    exCovX.get(), tmpHessian.get(), covDxdPi.get(),
    pointGradients.get(),
    mValidVoxelNumber);
  checkCudaErrors(cudaGetLastError());

  ComputeHessianListS2<<<grid, blockX>>>(transformedX.get(),
    transformedY.get(), transformedZ.get(), mValidPointsIndices.get(),
    mValidVoxelNumberPerValidPoint.get(), mVoxelIndices.get(), mValidPointsNumber,
    centroid.get(), centroid.get() + voxelNum, centroid.get() + 2 * voxelNum,
    mGaussD1, exCovX.get(),
    inverseCovariance.get(), inverseCovariance.get() + voxelNum,
    inverseCovariance.get() + 2 * voxelNum,
    inverseCovariance.get() + 3 * voxelNum,
    inverseCovariance.get() + 4 * voxelNum,
    inverseCovariance.get() + 5 * voxelNum,
    inverseCovariance.get() + 6 * voxelNum,
    inverseCovariance.get() + 7 * voxelNum,
    inverseCovariance.get() + 8 * voxelNum,
    pointHessians.get(), hessians.get());
  checkCudaErrors(cudaGetLastError());

  int fullSize = mValidPointsNumber;
  int halfSize = (fullSize - 1) / 2 + 1;
  while (fullSize > 1)
  {
    blockX = (halfSize > BLOCK_SIZE_X) ? BLOCK_SIZE_X : halfSize;
    gridX = (halfSize - 1) / blockX + 1;

    grid.x = gridX;
    grid.y = 6;
    grid.z = 6;
    MatrixSum<<<gridX, blockX>>>(hessians.get(),
      fullSize, halfSize, 6, 6, mValidPointsNumber);

    fullSize = halfSize;
    halfSize = (fullSize - 1) / 2 + 1;
  }

  checkCudaErrors(cudaDeviceSynchronize());

  MatrixDevice dhessian(6, 6, mValidPointsNumber, hessians.get());
  MatrixHost hhessian(6, 6);
  hhessian.MoveToHost(dhessian);

  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      hessian(i, j) = hhessian(i, j);
    }
  }

  dhessian.MemFree();
}

double GNormalDistributionsTransform::GetFitnessScore(double maxRange)
{
  double fitnessScore = 0.0;

  std::shared_ptr<float> transformedX = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> transformedY = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> transformedZ = AllocateCudaMemory<float>(mSourcePointsNumber);

  TransformPointCloud(mSourcePointsX, mSourcePointsY, mSourcePointsZ,
    transformedX, transformedY, transformedZ,
    mSourcePointsNumber, mFinalTransformationMatrix);

  std::shared_ptr<int> validDistance =
    AllocateCudaMemory<int>(mSourcePointsNumber);
  std::shared_ptr<double> minDistance =
    AllocateCudaMemory<double>(mSourcePointsNumber);

  mVoxelGrid.NearestNeighborSearch(transformedX, transformedY, transformedZ,
    mSourcePointsNumber, validDistance, minDistance, maxRange);

  int size = mSourcePointsNumber;
  int halfSize;

  while (size > 1)
  {
    halfSize = (size - 1) / 2 + 1;
    int blockX = (halfSize > BLOCK_SIZE_X) ? BLOCK_SIZE_X : halfSize;
    int gridX = (halfSize - 1) / blockX + 1;

    SummationUseGpu<double><<<gridX, blockX>>>(minDistance.get(), size, halfSize);
    checkCudaErrors(cudaGetLastError());

    SummationUseGpu<int><<<gridX, blockX>>>(validDistance.get(), size, halfSize);
    checkCudaErrors(cudaGetLastError());

    size = halfSize;
  }

  checkCudaErrors(cudaDeviceSynchronize());

  int nr;
  checkCudaErrors(cudaMemcpy(&nr, validDistance.get(),
    sizeof(int), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaMemcpy(&fitnessScore, minDistance.get(),
    sizeof(double), cudaMemcpyDeviceToHost));

  if (nr > 0)
    return (fitnessScore / nr);

  return DBL_MAX;
}

double GNormalDistributionsTransform::GetFitnessScoreWithRadiusSearch(
  double maxRange)
{
  double fitnessScore = 0.0;

  std::shared_ptr<float> transformedX = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> transformedY = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> transformedZ = AllocateCudaMemory<float>(mSourcePointsNumber);

  TransformPointCloud(mSourcePointsX, mSourcePointsY, mSourcePointsZ,
    transformedX, transformedY, transformedZ,
    mSourcePointsNumber, mFinalTransformationMatrix);

  std::shared_ptr<float> validTransformedX =
    AllocateCudaMemory<float>(mValidPointsNumber);
  std::shared_ptr<float> validTransformedY =
    AllocateCudaMemory<float>(mValidPointsNumber);
  std::shared_ptr<float> validTransformedZ =
    AllocateCudaMemory<float>(mValidPointsNumber);

  if (mValidPointsNumber > 0)
  {
    int size = mValidPointsNumber;
    int halfSize = (size - 1) / 2 + 1;

    int blockX = (halfSize > BLOCK_SIZE_X) ? BLOCK_SIZE_X : halfSize;
    int gridX = (halfSize - 1) / blockX + 1;

    ExtractValidPoints<<<gridX, blockX>>>(
      transformedX.get(), transformedY.get(), transformedZ.get(),
      validTransformedX.get(), validTransformedY.get(), validTransformedZ.get(),
      mValidPointsIndices.get(), mValidPointsNumber);
  }
  else
  {
    std::cout << "NormalDistributionTransform.cu: No Valid Points. Exiting "
      "getFitnessScoreWithRadiusSearch with 100.0." << std::endl;
    return 100.0;
  }

  std::shared_ptr<int> validDistance =
    AllocateCudaMemory<int>(mSourcePointsNumber);
  std::shared_ptr<double> minDistance =
    AllocateCudaMemory<double>(mSourcePointsNumber);

  mVoxelGrid.NearestNeighborSearch(
    validTransformedX, validTransformedY, validTransformedZ,
    mValidPointsNumber, validDistance, minDistance, maxRange);

  int size = mValidPointsNumber;
  int halfSize;

  while (size > 1)
  {
    halfSize = (size - 1) / 2 + 1;
    int blockX = (halfSize > BLOCK_SIZE_X) ? BLOCK_SIZE_X : halfSize;
    int gridX = (halfSize - 1) / blockX + 1;

    SummationUseGpu<double><<<gridX, blockX>>>(minDistance.get(), size, halfSize);
    checkCudaErrors(cudaGetLastError());

    SummationUseGpu<int><<<gridX, blockX>>>(validDistance.get(), size, halfSize);
    checkCudaErrors(cudaGetLastError());

    size = halfSize;
  }

  checkCudaErrors(cudaDeviceSynchronize());

  int nr;
  checkCudaErrors(cudaMemcpy(&nr, validDistance.get(),
    sizeof(int), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaMemcpy(&fitnessScore, minDistance.get(),
    sizeof(double), cudaMemcpyDeviceToHost));

  if (nr > 0)
    return (fitnessScore / nr);

  return DBL_MAX;
}

void GNormalDistributionsTransform::NearestPointsDistance(double maxRange)
{
  std::shared_ptr<float> transformedX = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> transformedY = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> transformedZ = AllocateCudaMemory<float>(mSourcePointsNumber);

  TransformPointCloud(mNearestPointsDistanceX, mNearestPointsDistanceY,
    mNearestPointsDistanceZ, transformedX, transformedY, transformedZ,
    mNearestPointsDistanceNumber, mFinalTransformationMatrix);

  std::shared_ptr<int> validDistance =
    AllocateCudaMemory<int>(mSourcePointsNumber);
  std::shared_ptr<double> minDistance =
    AllocateCudaMemory<double>(mSourcePointsNumber);

  mVoxelGrid.NearestNeighborSearch(transformedX, transformedY, transformedZ,
    mSourcePointsNumber, validDistance, minDistance, maxRange);

  checkCudaErrors(cudaDeviceSynchronize());
}

} // namespace gpu
