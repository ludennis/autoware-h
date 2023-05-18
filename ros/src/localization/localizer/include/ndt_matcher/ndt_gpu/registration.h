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

#ifndef GNDT_H_
#define GNDT_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <ndt_matcher/ndt_gpu/common.h>
#include <ndt_matcher/ndt_gpu/debug.h>
#include <ndt_matcher/ndt_gpu/matrix.h>
#include <ndt_matcher/ndt_gpu/matrix_host.h>
#include <ndt_matcher/ndt_gpu/matrix_device.h>
#include <ndt_matcher/ndt_gpu/memory.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace gpu
{

class GRegistration
{
public:
  GRegistration();
  GRegistration(const GRegistration &other) = delete;
  void Align(const Eigen::Matrix<float, 4, 4> &guess);
  void SetTransformationEpsilon(double transformationEpsilon);
  double GetTransformationEpsilon() const;
  void SetMaximumIterations(int maximumIterations);
  int GetMaximumIterations() const;
  Eigen::Matrix<float, 4, 4> GetFinalTransformation() const;

  /* Set input Scanned point cloud.
   * Copy input points from the main memory to the GPU memory */
  void SetInputSource(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
  void SetInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

  /* Set input reference map point cloud.
   * Copy input points from the main memory to the GPU memory */
  void SetInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
  void SetInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

  /* Set input points for calculating distance with nearest points */
  void SetInputNearestPointsDistanceSource(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

  int GetFinalNumIteration() const;
  bool HasConverged() const;
  virtual ~GRegistration(){};

protected:
  virtual void ComputeTransformation(const Eigen::Matrix<float, 4, 4> &guess);
  double mTransformationEpsilon;
  int mMaxIterations;
  /* Original scanned point clouds */
  int mSourcePointsNumber;
  std::shared_ptr<float> mSourcePointsX;
  std::shared_ptr<float> mSourcePointsY;
  std::shared_ptr<float> mSourcePointsZ;
  /* Transformed point clouds */
  std::shared_ptr<float> mTransformedPointsX;
  std::shared_ptr<float> mTransformedPointsY;
  std::shared_ptr<float> mTransformedPointsZ;
  /* Nearest points distance clouds */
  int mNearestPointsDistanceNumber;
  std::shared_ptr<float> mNearestPointsDistanceX;
  std::shared_ptr<float> mNearestPointsDistanceY;
  std::shared_ptr<float> mNearestPointsDistanceZ;
  std::shared_ptr<pcl::PointXYZ> mNearestPointsDistanceDevice;
  bool mConverged;
  int mNumberIteration;
  Eigen::Matrix<float, 4, 4> mFinalTransformationMatrix;
  Eigen::Matrix<float, 4, 4> mTransformationMatrix;
  Eigen::Matrix<float, 4, 4> mPreviousTransformationMatrix;
  bool mTargetPointCloudUpdated;
  /* Reference map point */
  int mTargetPointsNumber;
  std::shared_ptr<float> mTargetPointsX;
  std::shared_ptr<float> mTargetPointsY;
  std::shared_ptr<float> mTargetPointsZ;
};

} // namespace gpu

#endif // GNDT_H_
