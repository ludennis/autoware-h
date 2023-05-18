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

#include <ndt_matcher/ndt_gpu/registration.h>
#include <ndt_matcher/ndt_gpu/registration_kernel.cuh>

namespace gpu
{

static const int MAX_NEAREST_POINTS_DISTANCE_NUMBER = 60000;

GRegistration::GRegistration()
{
  mMaxIterations = 0;
  mSourcePointsX = mSourcePointsY = mSourcePointsZ = nullptr;
  mSourcePointsNumber = 0;
  mTransformedPointsX = mTransformedPointsY = mTransformedPointsZ = nullptr;
  mNearestPointsDistanceNumber = 0;
  mConverged = false;
  mNumberIteration = 0;
  mTransformationEpsilon = 0;
  mTargetPointCloudUpdated = true;
  mTargetPointsNumber = 0;
  mTargetPointsX = mTargetPointsY = mTargetPointsZ = nullptr;
  mNearestPointsDistanceX =
    AllocateCudaMemory<float>(MAX_NEAREST_POINTS_DISTANCE_NUMBER);
  mNearestPointsDistanceY =
    AllocateCudaMemory<float>(MAX_NEAREST_POINTS_DISTANCE_NUMBER);
  mNearestPointsDistanceZ =
    AllocateCudaMemory<float>(MAX_NEAREST_POINTS_DISTANCE_NUMBER);
  mNearestPointsDistanceDevice =
    AllocateCudaMemory<pcl::PointXYZ>(MAX_NEAREST_POINTS_DISTANCE_NUMBER);
}

void GRegistration::SetTransformationEpsilon(double transformationEpsilon)
{
  mTransformationEpsilon = transformationEpsilon;
}

double GRegistration::GetTransformationEpsilon() const
{
  return mTransformationEpsilon;
}

void GRegistration::SetMaximumIterations(int maximumIterations)
{
  mMaxIterations = maximumIterations;
}

int GRegistration::GetMaximumIterations() const
{
  return mMaxIterations;
}

Eigen::Matrix<float, 4, 4> GRegistration::GetFinalTransformation() const
{
  return mFinalTransformationMatrix;
}

int GRegistration::GetFinalNumIteration() const
{
  return mNumberIteration;
}

bool GRegistration::HasConverged() const
{
  return mConverged;
}

void GRegistration::SetInputSource(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
  if (input->size() > 0) {
    mSourcePointsNumber = input->size();

    std::shared_ptr<pcl::PointXYZI> tmp =
      AllocateCudaMemory<pcl::PointXYZI>(mSourcePointsNumber);
    pcl::PointXYZI *pointsFromHost = input->points.data();

    #ifndef __aarch64__
      checkCudaErrors(cudaHostRegister(pointsFromHost,
        sizeof(pcl::PointXYZI) * mSourcePointsNumber, cudaHostRegisterDefault));
    #endif
    checkCudaErrors(cudaMemcpy(tmp.get(),
      pointsFromHost, sizeof(pcl::PointXYZI) * mSourcePointsNumber,
      cudaMemcpyHostToDevice));

    mSourcePointsX = AllocateCudaMemory<float>(mSourcePointsNumber);
    mSourcePointsY = AllocateCudaMemory<float>(mSourcePointsNumber);
    mSourcePointsZ = AllocateCudaMemory<float>(mSourcePointsNumber);

    int blockX = (mSourcePointsNumber > BLOCK_SIZE_X) ?
      BLOCK_SIZE_X : mSourcePointsNumber;
    int gridX = (mSourcePointsNumber - 1) / blockX + 1;

    ConvertInputPoints<pcl::PointXYZI><<<gridX, blockX>>>(tmp.get(),
      mSourcePointsX.get(), mSourcePointsY.get(), mSourcePointsZ.get(),
      mSourcePointsNumber);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    mTransformedPointsX = AllocateCudaMemory<float>(mSourcePointsNumber);
    mTransformedPointsY = AllocateCudaMemory<float>(mSourcePointsNumber);
    mTransformedPointsZ = AllocateCudaMemory<float>(mSourcePointsNumber);

    checkCudaErrors(cudaMemcpy(mTransformedPointsX.get(),
      mSourcePointsX.get(), sizeof(float) * mSourcePointsNumber,
      cudaMemcpyDeviceToDevice));
    checkCudaErrors(cudaMemcpy(mTransformedPointsY.get(),
      mSourcePointsY.get(), sizeof(float) * mSourcePointsNumber,
      cudaMemcpyDeviceToDevice));
    checkCudaErrors(cudaMemcpy(mTransformedPointsZ.get(),
      mSourcePointsZ.get(), sizeof(float) * mSourcePointsNumber,
      cudaMemcpyDeviceToDevice));

    #ifndef __aarch64__
      checkCudaErrors(cudaHostUnregister(pointsFromHost));
    #endif
  }
}

void GRegistration::SetInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  if (input->size() > 0) {
    mSourcePointsNumber = input->size();

    std::shared_ptr<pcl::PointXYZ> tmp =
      AllocateCudaMemory<pcl::PointXYZ>(mSourcePointsNumber);
    pcl::PointXYZ *pointsFromHost = input->points.data();

    #ifndef __aarch64__
      checkCudaErrors(cudaHostRegister(pointsFromHost,
        sizeof(pcl::PointXYZ) * mSourcePointsNumber, cudaHostRegisterDefault));
    #endif
    checkCudaErrors(cudaMemcpy(tmp.get(),
      pointsFromHost, sizeof(pcl::PointXYZ) * mSourcePointsNumber,
      cudaMemcpyHostToDevice));

    mSourcePointsX = AllocateCudaMemory<float>(mSourcePointsNumber);
    mSourcePointsY = AllocateCudaMemory<float>(mSourcePointsNumber);
    mSourcePointsZ = AllocateCudaMemory<float>(mSourcePointsNumber);

    int blockX = (mSourcePointsNumber > BLOCK_SIZE_X) ?
      BLOCK_SIZE_X : mSourcePointsNumber;
    int gridX = (mSourcePointsNumber - 1) / blockX + 1;

    ConvertInputPoints<pcl::PointXYZ><<<gridX, blockX>>>(tmp.get(),
      mSourcePointsX.get(), mSourcePointsY.get(),
      mSourcePointsZ.get(), mSourcePointsNumber);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    mTransformedPointsX = AllocateCudaMemory<float>(mSourcePointsNumber);
    mTransformedPointsY = AllocateCudaMemory<float>(mSourcePointsNumber);
    mTransformedPointsZ = AllocateCudaMemory<float>(mSourcePointsNumber);

    checkCudaErrors(cudaMemcpy(mTransformedPointsX.get(),
      mSourcePointsX.get(), sizeof(float) * mSourcePointsNumber,
      cudaMemcpyDeviceToDevice));
    checkCudaErrors(cudaMemcpy(mTransformedPointsY.get(),
      mSourcePointsY.get(), sizeof(float) * mSourcePointsNumber,
      cudaMemcpyDeviceToDevice));
    checkCudaErrors(cudaMemcpy(mTransformedPointsZ.get(),
      mSourcePointsZ.get(), sizeof(float) * mSourcePointsNumber,
      cudaMemcpyDeviceToDevice));

    #ifndef __aarch64__
      checkCudaErrors(cudaHostUnregister(pointsFromHost));
    #endif
  }
}

void GRegistration::SetInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
  if (input->size() > 0) {
    mTargetPointsNumber = input->size();

    std::shared_ptr<pcl::PointXYZI> tmp =
      AllocateCudaMemory<pcl::PointXYZI>(mTargetPointsNumber);
    pcl::PointXYZI *pointsFromHost = input->points.data();

    #ifndef __aarch64__
      checkCudaErrors(cudaHostRegister(pointsFromHost,
        sizeof(pcl::PointXYZI) * mTargetPointsNumber, cudaHostRegisterDefault));
    #endif
    checkCudaErrors(cudaMemcpy(tmp.get(),
      pointsFromHost, sizeof(pcl::PointXYZI) * mTargetPointsNumber,
      cudaMemcpyHostToDevice));

    mTargetPointsX = AllocateCudaMemory<float>(mTargetPointsNumber);
    mTargetPointsY = AllocateCudaMemory<float>(mTargetPointsNumber);
    mTargetPointsZ = AllocateCudaMemory<float>(mTargetPointsNumber);

    int blockX = (mTargetPointsNumber > BLOCK_SIZE_X) ?
      BLOCK_SIZE_X : mTargetPointsNumber;
    int gridX = (mTargetPointsNumber - 1) / blockX + 1;

    ConvertInputPoints<pcl::PointXYZI><<<gridX, blockX>>>(tmp.get(),
      mTargetPointsX.get(), mTargetPointsY.get(), mTargetPointsZ.get(),
      mTargetPointsNumber);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    #ifndef __aarch64__
      checkCudaErrors(cudaHostUnregister(pointsFromHost));
    #endif
  }
}

void GRegistration::SetInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  PROFILE_CUDA_MEMORY_BEGIN(GRegistration_setInputTarget);
  if (input->size() > 0) {
    mTargetPointsNumber = input->size();

    std::shared_ptr<pcl::PointXYZ> tmp =
      AllocateCudaMemory<pcl::PointXYZ>(mTargetPointsNumber);
    pcl::PointXYZ *pointsFromHost = input->points.data();

    #ifndef __aarch64__
      checkCudaErrors(cudaHostRegister(pointsFromHost,
        sizeof(pcl::PointXYZ) * mTargetPointsNumber, cudaHostRegisterDefault));
    #endif
    checkCudaErrors(cudaMemcpy(tmp.get(),
      pointsFromHost, sizeof(pcl::PointXYZ) * mTargetPointsNumber,
      cudaMemcpyHostToDevice));

    mTargetPointsX = AllocateCudaMemory<float>(mTargetPointsNumber);
    mTargetPointsY = AllocateCudaMemory<float>(mTargetPointsNumber);
    mTargetPointsZ = AllocateCudaMemory<float>(mTargetPointsNumber);

    int blockX = (mTargetPointsNumber > BLOCK_SIZE_X) ?
      BLOCK_SIZE_X : mTargetPointsNumber;
    int gridX = (mTargetPointsNumber - 1) / blockX + 1;

    ConvertInputPoints<pcl::PointXYZ><<<gridX, blockX>>>(tmp.get(),
      mTargetPointsX.get(), mTargetPointsY.get(), mTargetPointsZ.get(),
      mTargetPointsNumber);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    #ifndef __aarch64__
      checkCudaErrors(cudaHostUnregister(pointsFromHost));
    #endif
  }
  PROFILE_CUDA_MEMORY_END(GRegistration_setInputTarget);
}

void GRegistration::SetInputNearestPointsDistanceSource(
  pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  if (input->size() > 0) {
    mNearestPointsDistanceNumber = input->size();
    pcl::PointXYZ *pointsFromHost = input->points.data();

    #ifndef __aarch64__
      checkCudaErrors(cudaHostRegister(pointsFromHost,
        sizeof(pcl::PointXYZ) * mNearestPointsDistanceNumber,
        cudaHostRegisterDefault));
    #endif

    if (mNearestPointsDistanceNumber > MAX_NEAREST_POINTS_DISTANCE_NUMBER)
    {
      throw std::runtime_error("Number of points, used for semantic "
      "segmentation, exceeds the limit.");
    }

    checkCudaErrors(cudaMemcpy(mNearestPointsDistanceDevice.get(),
      pointsFromHost, sizeof(pcl::PointXYZ) * mNearestPointsDistanceNumber,
      cudaMemcpyHostToDevice));

    int blockX = (mNearestPointsDistanceNumber > BLOCK_SIZE_X) ?
      BLOCK_SIZE_X : mNearestPointsDistanceNumber;
    int gridX = (mNearestPointsDistanceNumber - 1) / blockX + 1;

    ConvertInputPoints<pcl::PointXYZ><<<gridX, blockX>>>(
      mNearestPointsDistanceDevice.get(), mNearestPointsDistanceX.get(),
      mNearestPointsDistanceY.get(), mNearestPointsDistanceZ.get(),
      mNearestPointsDistanceNumber);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    #ifndef __aarch64__
      checkCudaErrors(cudaHostUnregister(pointsFromHost));
    #endif
  }
}

void GRegistration::Align(const Eigen::Matrix<float, 4, 4> &guess)
{
  mConverged = false;
  mFinalTransformationMatrix =
    mTransformationMatrix = mPreviousTransformationMatrix =
    Eigen::Matrix<float, 4, 4>::Identity();

  ComputeTransformation(guess);
}

void GRegistration::ComputeTransformation(const Eigen::Matrix<float, 4, 4> &guess)
{
  printf("Unsupported by Registration\n");
}

} // namespace gpu
