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

#include <ndt_matcher/ndt_gpu/common.h>
#include <ndt_matcher/ndt_gpu/debug.h>
#include <ndt_matcher/ndt_gpu/memory.h>
#include <ndt_matcher/ndt_gpu/symmetric_eigen_solver.h>
#include <ndt_matcher/ndt_gpu/voxel_grid.h>
#include <ndt_matcher/ndt_gpu/voxel_grid_kernel.cuh>

#include <cmath>
#include <inttypes.h>
#include <limits>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <thrust/copy.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/fill.h>
#include <thrust/scan.h>
#include <vector>

namespace gpu
{

GVoxelGrid::GVoxelGrid():
  mSourcePointsX(nullptr),
  mSourcePointsY(nullptr),
  mSourcePointsZ(nullptr),
  mSourcePointsNumber(0),
  mVoxelGridCentroid(nullptr),
  mVoxelGridCovariance(nullptr),
  mVoxelGridInverseCovariance(nullptr),
  mPointsNumberPerVoxelGrid(nullptr),
  mVoxelGridNumber(0),
  mVoxelGridUpperBoundMaxX(FLT_MAX),
  mVoxelGridUpperBoundMaxY(FLT_MAX),
  mVoxelGridUpperBoundMaxZ(FLT_MAX),
  mVoxelGridLowerBoundMinX(FLT_MIN),
  mVoxelGridLowerBoundMinY(FLT_MIN),
  mVoxelGridLowerBoundMinZ(FLT_MIN),
  mVoxelGridLeafSizeX(0),
  mVoxelGridLeafSizeY(0),
  mVoxelGridLeafSizeZ(0),
  mVoxelGridNumberUpperBoundMaxX(0),
  mVoxelGridNumberUpperBoundMaxY(0),
  mVoxelGridNumberUpperBoundMaxZ(0),
  mVoxelGridNumberLowerBoundMinX(0),
  mVoxelGridNumberLowerBoundMinY(0),
  mVoxelGridNumberLowerBoundMinZ(0),
  mVoxelGridNumberX(0),
  mVoxelGridNumberY(0),
  mVoxelGridNumberZ(0),
  mMinPointsNumberPerVoxelGrid(6),
  mSourcePointsIndicesGroupedByVoxelGrid(nullptr),
  mSourcePointsIndex(nullptr)
{};

GVoxelGrid::~GVoxelGrid() {
  mOctreeCentroids.clear();
  mOctreeGridSize.clear();
}

void GVoxelGrid::Initialize()
{
  mVoxelGridCentroid = AllocateCudaMemory<double>(mVoxelGridNumber * 3);
  mVoxelGridCovariance = AllocateCudaMemory<double>(mVoxelGridNumber * 9);
  mVoxelGridInverseCovariance = AllocateCudaMemory<double>(mVoxelGridNumber * 9);
  mPointsNumberPerVoxelGrid = AllocateCudaMemory<int>(mVoxelGridNumber);

  checkCudaErrors(cudaMemset(mVoxelGridInverseCovariance.get(), 0,
    sizeof(double) * 9 * mVoxelGridNumber));
  checkCudaErrors(cudaMemset(mPointsNumberPerVoxelGrid.get(), 0,
    sizeof(int) * mVoxelGridNumber));
  checkCudaErrors(cudaDeviceSynchronize());
}

int GVoxelGrid::GetVoxelNum() const
{
  return mVoxelGridNumber;
}

float GVoxelGrid::GetMaxX() const
{
  return mVoxelGridUpperBoundMaxX;
}

float GVoxelGrid::GetMaxY() const
{
  return mVoxelGridUpperBoundMaxY;
}

float GVoxelGrid::GetMaxZ() const
{
  return mVoxelGridUpperBoundMaxZ;
}

float GVoxelGrid::GetMinX() const
{
  return mVoxelGridLowerBoundMinX;
}

float GVoxelGrid::GetMinY() const
{
  return mVoxelGridLowerBoundMinY;
}

float GVoxelGrid::GetMinZ() const
{
  return mVoxelGridLowerBoundMinZ;
}

float GVoxelGrid::GetVoxelX() const
{
  return mVoxelGridLeafSizeX;
}

float GVoxelGrid::GetVoxelY() const
{
  return mVoxelGridLeafSizeY;
}

float GVoxelGrid::GetVoxelZ() const
{
  return mVoxelGridLeafSizeZ;
}

int GVoxelGrid::GetMaxBX() const
{
  return mVoxelGridNumberUpperBoundMaxX;
}

int GVoxelGrid::GetMaxBY() const
{
  return mVoxelGridNumberUpperBoundMaxY;
}

int GVoxelGrid::GetMaxBZ() const
{
  return mVoxelGridNumberUpperBoundMaxZ;
}

int GVoxelGrid::GetMinBX() const
{
  return mVoxelGridNumberLowerBoundMinX;
}

int GVoxelGrid::GetMinBY() const
{
  return mVoxelGridNumberLowerBoundMinY;
}

int GVoxelGrid::GetMinBZ() const
{
  return mVoxelGridNumberLowerBoundMinZ;
}

int GVoxelGrid::GetVgridX() const
{
  return mVoxelGridNumberX;
}

int GVoxelGrid::GetVgridY() const
{
  return mVoxelGridNumberY;
}

int GVoxelGrid::GetVgridZ() const
{
  return mVoxelGridNumberZ;
}

void GVoxelGrid::SetLeafSize(float leafSizeX, float leafSizeY, float leafSizeZ)
{
  mVoxelGridLeafSizeX = leafSizeX;
  mVoxelGridLeafSizeY = leafSizeY;
  mVoxelGridLeafSizeZ = leafSizeZ;
}

std::shared_ptr<double> GVoxelGrid::GetCentroidList() const
{
  return mVoxelGridCentroid;
}

std::shared_ptr<double> GVoxelGrid::GetCovarianceList() const
{
  return mVoxelGridCovariance;
}

std::shared_ptr<double> GVoxelGrid::GetInverseCovarianceList() const
{
  return mVoxelGridInverseCovariance;
}

std::shared_ptr<int> GVoxelGrid::GetPointsPerVoxelList() const
{
  return mPointsNumberPerVoxelGrid;
}

const std::vector<int> & GVoxelGrid::GetValidPointsIndices() const
{
  return mValidPointsIndices;
}

const std::vector<double> & GVoxelGrid::GetMinDistances() const
{
  return mOutputMinDistances;
}

void GVoxelGrid::ComputeCentroidAndCovariance()
{
  int blockX = (mVoxelGridNumber > BLOCK_SIZE_X) ? BLOCK_SIZE_X : mVoxelGridNumber;
  int gridX = (mVoxelGridNumber - 1) / blockX + 1;

  InitCentroidAndCovariance<<<gridX, blockX>>>(mSourcePointsX.get(),
    mSourcePointsY.get(), mSourcePointsZ.get(),
    mSourcePointsIndicesGroupedByVoxelGrid.get(),
    mSourcePointsIndex.get(), mVoxelGridCentroid.get(),
    mVoxelGridCovariance.get(), mVoxelGridNumber);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());

  std::shared_ptr<double> pointsSum =
    AllocateCudaMemory<double>(mVoxelGridNumber * 3);
  checkCudaErrors(cudaMemcpy(pointsSum.get(), mVoxelGridCentroid.get(),
    sizeof(double) * mVoxelGridNumber * 3, cudaMemcpyDeviceToDevice));

  UpdateVoxelCentroid<<<gridX, blockX>>>(mVoxelGridCentroid.get(),
    mPointsNumberPerVoxelGrid.get(), mVoxelGridNumber);
  checkCudaErrors(cudaGetLastError());

  UpdateVoxelCovariance<<<gridX, blockX>>>(mVoxelGridCentroid.get(),
    pointsSum.get(), mVoxelGridCovariance.get(),
    mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());

  std::shared_ptr<double> eigenvaluesDev =
    AllocateCudaMemory<double>(mVoxelGridNumber * 3);
  std::shared_ptr<double> eigenvectorsDev =
    AllocateCudaMemory<double>(mVoxelGridNumber * 9);

  SymmetricEigensolver3x3 sv(mVoxelGridNumber);

  sv.SetInputMatrices(mVoxelGridCovariance.get());
  sv.SetEigenvalues(eigenvaluesDev.get());
  sv.SetEigenvectors(eigenvectorsDev.get());

  Normalize<<<gridX, blockX>>>(sv, mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  ComputeEigenvalues<<<gridX, blockX>>>(sv, mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  ComputeEvec00<<<gridX, blockX>>>(sv, mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  ComputeEvec01<<<gridX, blockX>>>(sv, mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  ComputeEvec10<<<gridX, blockX>>>(sv, mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  ComputeEvec11<<<gridX, blockX>>>(sv, mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  ComputeEvec2<<<gridX, blockX>>>(sv, mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  UpdateEval<<<gridX, blockX>>>(sv, mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  UpdateEval2<<<gridX, blockX>>>(eigenvaluesDev.get(),
    mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  ComputeInverseEigenvectors<<<gridX, blockX>>>(mVoxelGridInverseCovariance.get(),
    mPointsNumberPerVoxelGrid.get(), mVoxelGridNumber,
    eigenvectorsDev.get(), mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  UpdateCovarianceS0<<<gridX, blockX>>>(mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, eigenvaluesDev.get(),
    eigenvectorsDev.get(), mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());

  for (int i = 0; i < 3; i++)
  {
    UpdateCovarianceS1<<<gridX, blockX>>>(mVoxelGridCovariance.get(),
      mVoxelGridInverseCovariance.get(), mPointsNumberPerVoxelGrid.get(),
      mVoxelGridNumber, eigenvectorsDev.get(), mMinPointsNumberPerVoxelGrid, i);
    checkCudaErrors(cudaGetLastError());
  }
  checkCudaErrors(cudaDeviceSynchronize());

  ComputeInverseCovariance<<<gridX, blockX>>>(mVoxelGridCovariance.get(),
    mVoxelGridInverseCovariance.get(), mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mMinPointsNumberPerVoxelGrid);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());

  sv.MemFree();
}

void GVoxelGrid::SetInput(std::shared_ptr<float> &x,
  std::shared_ptr<float> &y, std::shared_ptr<float> &z, int pointsNumber)
{
  PROFILE_CUDA_MEMORY_BEGIN(GVoxelGrid_setInput);
  if (pointsNumber <= 0)
  {
    return;
  }
  mSourcePointsX = x;
  mSourcePointsY = y;
  mSourcePointsZ = z;
  mSourcePointsNumber = pointsNumber;

  FindBoundaries();
  mVoxelGridNumber = mVoxelGridNumberX * mVoxelGridNumberY * mVoxelGridNumberZ;
  Initialize();
  ScatterPointsToVoxelGrid();
  ComputeCentroidAndCovariance();
  BuildOctree();
  PROFILE_CUDA_MEMORY_END(GVoxelGrid_setInput);
}

void GVoxelGrid::FindBoundaries()
{
  std::shared_ptr<float> maxX = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> maxY = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> maxZ = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> minX = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> minY = AllocateCudaMemory<float>(mSourcePointsNumber);
  std::shared_ptr<float> minZ = AllocateCudaMemory<float>(mSourcePointsNumber);

  checkCudaErrors(cudaMemcpy(maxX.get(), mSourcePointsX.get(),
    sizeof(float) * mSourcePointsNumber, cudaMemcpyDeviceToDevice));
  checkCudaErrors(cudaMemcpy(maxY.get(), mSourcePointsY.get(),
    sizeof(float) * mSourcePointsNumber, cudaMemcpyDeviceToDevice));
  checkCudaErrors(cudaMemcpy(maxZ.get(), mSourcePointsZ.get(),
    sizeof(float) * mSourcePointsNumber, cudaMemcpyDeviceToDevice));

  checkCudaErrors(cudaMemcpy(minX.get(), mSourcePointsX.get(),
    sizeof(float) * mSourcePointsNumber, cudaMemcpyDeviceToDevice));
  checkCudaErrors(cudaMemcpy(minY.get(), mSourcePointsY.get(),
    sizeof(float) * mSourcePointsNumber, cudaMemcpyDeviceToDevice));
  checkCudaErrors(cudaMemcpy(minZ.get(), mSourcePointsZ.get(),
    sizeof(float) * mSourcePointsNumber, cudaMemcpyDeviceToDevice));

  int pointsNumber = mSourcePointsNumber;

  while (pointsNumber > 1)
  {
    int halfPointsNumber = (pointsNumber - 1) / 2 + 1;
    int blockX = (halfPointsNumber > BLOCK_SIZE_X) ? BLOCK_SIZE_X : halfPointsNumber;
    int gridX = (halfPointsNumber - 1) / blockX + 1;

    FindMax<<<gridX, blockX>>>(maxX.get(),
      maxY.get(), maxZ.get(), pointsNumber, halfPointsNumber);
    checkCudaErrors(cudaGetLastError());

    FindMin<<<gridX, blockX>>>(minX.get(),
      minY.get(), minZ.get(), pointsNumber, halfPointsNumber);
    checkCudaErrors(cudaGetLastError());

    pointsNumber = halfPointsNumber;
  }

  checkCudaErrors(cudaDeviceSynchronize());

  checkCudaErrors(cudaMemcpy(&mVoxelGridUpperBoundMaxX, maxX.get(),
    sizeof(float), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaMemcpy(&mVoxelGridUpperBoundMaxY, maxY.get(),
    sizeof(float), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaMemcpy(&mVoxelGridUpperBoundMaxZ, maxZ.get(),
    sizeof(float), cudaMemcpyDeviceToHost));

  checkCudaErrors(cudaMemcpy(&mVoxelGridLowerBoundMinX, minX.get(),
    sizeof(float), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaMemcpy(&mVoxelGridLowerBoundMinY, minY.get(),
    sizeof(float), cudaMemcpyDeviceToHost));
  checkCudaErrors(cudaMemcpy(&mVoxelGridLowerBoundMinZ, minZ.get(),
    sizeof(float), cudaMemcpyDeviceToHost));

  mVoxelGridNumberUpperBoundMaxX =
    static_cast<int> (floor(mVoxelGridUpperBoundMaxX / mVoxelGridLeafSizeX));
  mVoxelGridNumberUpperBoundMaxY =
    static_cast<int> (floor(mVoxelGridUpperBoundMaxY / mVoxelGridLeafSizeY));
  mVoxelGridNumberUpperBoundMaxZ =
    static_cast<int> (floor(mVoxelGridUpperBoundMaxZ / mVoxelGridLeafSizeZ));

  mVoxelGridNumberLowerBoundMinX =
    static_cast<int> (floor(mVoxelGridLowerBoundMinX / mVoxelGridLeafSizeX));
  mVoxelGridNumberLowerBoundMinY =
    static_cast<int> (floor(mVoxelGridLowerBoundMinY / mVoxelGridLeafSizeY));
  mVoxelGridNumberLowerBoundMinZ =
    static_cast<int> (floor(mVoxelGridLowerBoundMinZ / mVoxelGridLeafSizeZ));

  mVoxelGridNumberX =
    mVoxelGridNumberUpperBoundMaxX - mVoxelGridNumberLowerBoundMinX + 1;
  mVoxelGridNumberY =
    mVoxelGridNumberUpperBoundMaxY - mVoxelGridNumberLowerBoundMinY + 1;
  mVoxelGridNumberZ =
    mVoxelGridNumberUpperBoundMaxZ - mVoxelGridNumberLowerBoundMinZ + 1;
}

template <typename T>
void GVoxelGrid::ExclusiveScan(T *input, int eleNum, T & sum)
{
  thrust::device_ptr<T> devPtr(input);

  thrust::exclusive_scan(devPtr, devPtr + eleNum, devPtr);
  checkCudaErrors(cudaDeviceSynchronize());

  sum = *(devPtr + eleNum - 1);
}

template <typename T>
void GVoxelGrid::ExclusiveScan(T *input, int eleNum)
{
  thrust::device_ptr<T> devPtr(input);

  thrust::exclusive_scan(devPtr, devPtr + eleNum, devPtr);
  checkCudaErrors(cudaDeviceSynchronize());
}

void GVoxelGrid::RadiusSearch(std::shared_ptr<float> &queryPointsX,
  std::shared_ptr<float> &queryPointsY,
  std::shared_ptr<float> &queryPointsZ,
  int pointsNum, float radius, int maxNn,
  std::shared_ptr<int> &validPointsIndex,
  std::shared_ptr<int> &validVoxelNumPerValidPoint,
  std::shared_ptr<int> &validVoxelId,
  int &validVoxelNum, int &validPointsNum)
{
  int blockX = (pointsNum > BLOCK_SIZE_X) ? BLOCK_SIZE_X : pointsNum;
  int gridX = (pointsNum - 1) / blockX + 1;

  std::shared_ptr<int> maxVoxelIndexX = AllocateCudaMemory<int>(pointsNum);
  std::shared_ptr<int> maxVoxelIndexY = AllocateCudaMemory<int>(pointsNum);
  std::shared_ptr<int> maxVoxelIndexZ = AllocateCudaMemory<int>(pointsNum);
  std::shared_ptr<int> minVoxelIndexX = AllocateCudaMemory<int>(pointsNum);
  std::shared_ptr<int> minVoxelIndexY = AllocateCudaMemory<int>(pointsNum);
  std::shared_ptr<int> minVoxelIndexZ = AllocateCudaMemory<int>(pointsNum);

  int totalCandidateVoxelNum;

  std::shared_ptr<int> candidateVoxelNumPerPoint =
    AllocateCudaMemory<int>((pointsNum + 1));

  FindBoundariesOfCandidateVoxels<<<gridX, blockX>>>(queryPointsX.get(),
    queryPointsY.get(), queryPointsZ.get(), radius, pointsNum,
    mVoxelGridLeafSizeX, mVoxelGridLeafSizeY, mVoxelGridLeafSizeZ,
    mVoxelGridNumberUpperBoundMaxX, mVoxelGridNumberUpperBoundMaxY, mVoxelGridNumberUpperBoundMaxZ,
    mVoxelGridNumberLowerBoundMinX, mVoxelGridNumberLowerBoundMinY, mVoxelGridNumberLowerBoundMinZ,
    maxVoxelIndexX.get(), maxVoxelIndexY.get(), maxVoxelIndexZ.get(),
    minVoxelIndexX.get(), minVoxelIndexY.get(), minVoxelIndexZ.get(),
    candidateVoxelNumPerPoint.get());
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());
  ExclusiveScan(candidateVoxelNumPerPoint.get(),
    pointsNum + 1, totalCandidateVoxelNum);

  if (totalCandidateVoxelNum <= 0)
  {
    std::cout << "No candidate voxel was found. Exiting..." << std::endl;

    validPointsIndex = nullptr;
    validVoxelNumPerValidPoint = nullptr;
    validVoxelId = nullptr;

    validVoxelNum = 0;
    validPointsNum = 0;

    return;
  }

  std::shared_ptr<int> candidateVoxelId =
    AllocateCudaMemory<int>(totalCandidateVoxelNum);

  UpdateCandidateVoxelIds<<<gridX, blockX>>>(pointsNum,
    mVoxelGridNumberX, mVoxelGridNumberY, mVoxelGridNumberZ,
    maxVoxelIndexX.get(), maxVoxelIndexY.get(), maxVoxelIndexZ.get(),
    minVoxelIndexX.get(), minVoxelIndexY.get(), minVoxelIndexZ.get(),
    candidateVoxelNumPerPoint.get(), candidateVoxelId.get());
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());

  std::shared_ptr<int> validVoxelMark =
    AllocateCudaMemory<int>(totalCandidateVoxelNum);
  std::shared_ptr<int> validVoxelCount = AllocateCudaMemory<int>((pointsNum + 1));
  std::shared_ptr<int> validPointsMark = AllocateCudaMemory<int>(pointsNum);

  blockX = (totalCandidateVoxelNum > BLOCK_SIZE_X) ?
    BLOCK_SIZE_X : totalCandidateVoxelNum;
  gridX = (totalCandidateVoxelNum - 1) / blockX + 1;

  InspectCandidateVoxels<<<gridX, blockX>>>(queryPointsX.get(),
    queryPointsY.get(), queryPointsZ.get(), radius, maxNn, pointsNum,
    mVoxelGridCentroid.get(), mPointsNumberPerVoxelGrid.get(), mVoxelGridNumber,
    candidateVoxelNumPerPoint.get(), candidateVoxelId.get(),
    validVoxelMark.get(), validVoxelCount.get(), validPointsMark.get());
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());

  std::shared_ptr<int> validPointsLocation =
    AllocateCudaMemory<int>((pointsNum + 1));

  checkCudaErrors(cudaMemset(validPointsLocation.get(),
    0, sizeof(int) * (pointsNum + 1)));
  checkCudaErrors(cudaMemcpy(validPointsLocation.get(), validPointsMark.get(),
    sizeof(int) * pointsNum, cudaMemcpyDeviceToDevice));

  ExclusiveScan(validPointsLocation.get(), pointsNum + 1, validPointsNum);

  if (validPointsNum <= 0)
  {
    std::cout << "No valid point was found. Exiting...: " << validPointsNum << std::endl;

    validPointsIndex = nullptr;
    validVoxelNumPerValidPoint = nullptr;
    validVoxelId = nullptr;

    validVoxelNum = 0;
    validPointsNum = 0;

    return;
  }

  validPointsIndex = AllocateCudaMemory<int>(validPointsNum);

  CollectValidPoints<<<gridX, blockX>>>(validPointsMark.get(),
    validPointsIndex.get(), validPointsLocation.get(), pointsNum);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());

  mValidPointsIndices.resize(validPointsNum);
  checkCudaErrors(cudaMemcpy(mValidPointsIndices.data(), validPointsIndex.get(),
    sizeof(int) * mValidPointsIndices.size(), cudaMemcpyDeviceToHost));

  validVoxelNumPerValidPoint = AllocateCudaMemory<int>((validPointsNum + 1));

  CollectValidVoxelCount<<<gridX, blockX>>>(validVoxelCount.get(),
    validVoxelNumPerValidPoint.get(), validPointsLocation.get(), pointsNum);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());

  ExclusiveScan(validVoxelNumPerValidPoint.get(), validPointsNum + 1, validVoxelNum);

  std::shared_ptr<int> validVoxelLocation =
    AllocateCudaMemory<int>((totalCandidateVoxelNum + 1));
  checkCudaErrors(cudaMemcpy(validVoxelLocation.get(), validVoxelMark.get(),
    sizeof(int) * totalCandidateVoxelNum, cudaMemcpyDeviceToDevice));

  ExclusiveScan(validVoxelLocation.get(),
    totalCandidateVoxelNum + 1, validVoxelNum);

  if (validVoxelNum <= 0)
  {
    validPointsIndex = nullptr;
    validVoxelNumPerValidPoint = nullptr;
    validVoxelId = nullptr;

    validVoxelNum = 0;
    validPointsNum = 0;
  }

  validVoxelId = AllocateCudaMemory<int>(validVoxelNum);

  blockX = (totalCandidateVoxelNum > BLOCK_SIZE_X) ?
    BLOCK_SIZE_X : totalCandidateVoxelNum;
  gridX = (totalCandidateVoxelNum - 1) / blockX + 1;

  CollectValidVoxels<<<gridX, blockX>>>(validVoxelMark.get(),
    candidateVoxelId.get(), validVoxelId.get(),
    validVoxelLocation.get(), totalCandidateVoxelNum);

  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());
}

void GVoxelGrid::ScatterPointsToVoxelGrid()
{
  int blockX = (mSourcePointsNumber > BLOCK_SIZE_X) ? BLOCK_SIZE_X : mSourcePointsNumber;
  int gridX = (mSourcePointsNumber - 1) / blockX + 1;

  InsertPointsToGrid<<<gridX, blockX>>>(mSourcePointsX.get(),
    mSourcePointsY.get(), mSourcePointsZ.get(), mSourcePointsNumber,
    mPointsNumberPerVoxelGrid.get(),
    mVoxelGridNumber, mVoxelGridNumberX, mVoxelGridNumberY, mVoxelGridNumberZ,
    mVoxelGridLeafSizeX, mVoxelGridLeafSizeY, mVoxelGridLeafSizeZ,
    mVoxelGridNumberLowerBoundMinX, mVoxelGridNumberLowerBoundMinY,
    mVoxelGridNumberLowerBoundMinZ);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());

  mSourcePointsIndicesGroupedByVoxelGrid =
    AllocateCudaMemory<int>((mVoxelGridNumber + 1));
  std::shared_ptr<int> writingLocation =
    AllocateCudaMemory<int>(mVoxelGridNumber);

  checkCudaErrors(cudaMemcpy(mSourcePointsIndicesGroupedByVoxelGrid.get(),
    mPointsNumberPerVoxelGrid.get(),
    sizeof(int) * mVoxelGridNumber, cudaMemcpyDeviceToDevice));

  ExclusiveScan(mSourcePointsIndicesGroupedByVoxelGrid.get(), mVoxelGridNumber + 1);

  checkCudaErrors(cudaMemcpy(writingLocation.get(),
    mSourcePointsIndicesGroupedByVoxelGrid.get(), sizeof(int) * mVoxelGridNumber,
    cudaMemcpyDeviceToDevice));

  mSourcePointsIndex = AllocateCudaMemory<int>(mSourcePointsNumber);

  ScatterPointsToVoxels<<<gridX, blockX>>>(mSourcePointsX.get(),
    mSourcePointsY.get(), mSourcePointsZ.get(),
    mSourcePointsNumber, mVoxelGridNumber,
    mVoxelGridLeafSizeX, mVoxelGridLeafSizeY, mVoxelGridLeafSizeZ,
    mVoxelGridNumberLowerBoundMinX, mVoxelGridNumberLowerBoundMinY,
    mVoxelGridNumberLowerBoundMinZ,
    mVoxelGridNumberX, mVoxelGridNumberY, mVoxelGridNumberZ,
    writingLocation.get(), mSourcePointsIndex.get());
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());
}

void GVoxelGrid::BuildOctree()
{
  mOctreeCentroids.clear();
  mOctreeGridSize.clear();

  double minX = mVoxelGridLowerBoundMinX;
  double minY = mVoxelGridLowerBoundMinY;
  double minZ = mVoxelGridLowerBoundMinZ;
  double leafSizeX = mVoxelGridLeafSizeX;
  double leafSizeY = mVoxelGridLeafSizeY;
  double leafSizeZ = mVoxelGridLeafSizeZ;

  mOctreeCentroids.push_back(mVoxelGridCentroid);
  OctreeGridSize gridSize;

  gridSize.size_x = mVoxelGridNumberX;
  gridSize.size_y = mVoxelGridNumberY;
  gridSize.size_z = mVoxelGridNumberZ;

  mOctreeGridSize.push_back(gridSize);

  int nodeNumber = mVoxelGridNumber;
  int childGridX, childGridY, childGridZ;
  int parentGridX, parentGridY, parentGridZ;

  int i = 0;

  while (nodeNumber > 8)
  {
    childGridX = mOctreeGridSize[i].size_x;
    childGridY = mOctreeGridSize[i].size_y;
    childGridZ = mOctreeGridSize[i].size_z;
    parentGridX = (childGridX - 1) / 2 + 1;
    parentGridY = (childGridY - 1) / 2 + 1;
    parentGridZ = (childGridZ - 1) / 2 + 1;
    nodeNumber = parentGridX * parentGridY * parentGridZ;

    std::shared_ptr<double> parentCentroids =
      AllocateCudaMemory<double>(nodeNumber * 3);

    std::shared_ptr<double> childCentroids = mOctreeCentroids[i];

    int blockX = (parentGridX > BLOCK_X) ? BLOCK_X : parentGridX;
    int blockY = (parentGridY > BLOCK_Y) ? BLOCK_Y : parentGridY;
    int blockZ = (parentGridZ > BLOCK_Z) ? BLOCK_Z : parentGridZ;
    int gridX = (parentGridX - 1) / blockX + 1;
    int gridY = (parentGridY - 1) / blockY + 1;
    int gridZ = (parentGridZ - 1) / blockZ + 1;

    dim3 block(blockX, blockY, blockZ);
    dim3 grid(gridX, gridY, gridZ);

    BuildOctreeParents<<<grid, block>>>(
      childGridX,
      childGridY,
      childGridZ,
      parentCentroids.get(),
      parentGridX,
      parentGridY,
      parentGridZ,
      minX,
      minY,
      minZ,
      leafSizeX,
      leafSizeY,
      leafSizeZ,
      i);
    checkCudaErrors(cudaGetLastError());
    mOctreeCentroids.push_back(parentCentroids);

    gridSize.size_x = parentGridX;
    gridSize.size_y = parentGridY;
    gridSize.size_z = parentGridZ;

    mOctreeGridSize.push_back(gridSize);
    i++;
  }

  checkCudaErrors(cudaDeviceSynchronize());
}

void GVoxelGrid::NearestNeighborSearch(std::shared_ptr<float> &transformedX,
  std::shared_ptr<float> &transformedY, std::shared_ptr<float> &transformedZ,
  int pointsNumber, std::shared_ptr<int> &validDistance,
  std::shared_ptr<double> &minDistance, float maxRange)
{
  mOutputMinDistances.clear();

  std::shared_ptr<int> xVoxelIndexPerPoints = AllocateCudaMemory<int>(pointsNumber);
  std::shared_ptr<int> yVoxelIndexPerPoints = AllocateCudaMemory<int>(pointsNumber);
  std::shared_ptr<int> zVoxelIndexPerPoints = AllocateCudaMemory<int>(pointsNumber);
  checkCudaErrors(cudaMemset(xVoxelIndexPerPoints.get(),
    0, sizeof(int) * pointsNumber));
  checkCudaErrors(cudaMemset(yVoxelIndexPerPoints.get(),
    0, sizeof(int) * pointsNumber));
  checkCudaErrors(cudaMemset(zVoxelIndexPerPoints.get(),
    0, sizeof(int) * pointsNumber));
  checkCudaErrors(cudaDeviceSynchronize());

  int blockX = (pointsNumber > BLOCK_SIZE_X) ? BLOCK_SIZE_X : pointsNumber;
  int gridX = (pointsNumber - 1) / blockX + 1;

  for (int i = mOctreeCentroids.size() - 1; i >= 0; i--)
  {
    std::shared_ptr<double> centroids = mOctreeCentroids[i];
    int voxelGridNumX = mOctreeGridSize[i].size_x;
    int voxelGridNumY = mOctreeGridSize[i].size_y;
    int voxelGridNumZ = mOctreeGridSize[i].size_z;
    int nodeNum = voxelGridNumX * voxelGridNumY * voxelGridNumZ;

    NearestOctreeNodeSearch<<<gridX, blockX>>>(
      transformedX.get(), transformedY.get(), transformedZ.get(),
      xVoxelIndexPerPoints.get(), yVoxelIndexPerPoints.get(),
      zVoxelIndexPerPoints.get(),
      pointsNumber, centroids.get(),
      voxelGridNumX, voxelGridNumY, voxelGridNumZ, nodeNum, i);
    checkCudaErrors(cudaGetLastError());
  }

  NearestPointSearch<<<gridX, blockX>>>(transformedX.get(),
    transformedY.get(), transformedZ.get(), pointsNumber,
    mSourcePointsX.get(), mSourcePointsY.get(), mSourcePointsZ.get(),
    xVoxelIndexPerPoints.get(), yVoxelIndexPerPoints.get(), zVoxelIndexPerPoints.get(),
    mVoxelGridNumberX, mVoxelGridNumberY, mVoxelGridNumberZ,
    mSourcePointsIndicesGroupedByVoxelGrid.get(), mSourcePointsIndex.get(),
    minDistance.get());
  checkCudaErrors(cudaGetLastError());

  mOutputMinDistances.resize(pointsNumber);
  checkCudaErrors(cudaMemcpy(
    mOutputMinDistances.data(),
    minDistance.get(),
    sizeof(double)*mOutputMinDistances.size(),
    cudaMemcpyDeviceToHost));

  VerifyDistances<<<gridX, blockX>>>(validDistance.get(),
    minDistance.get(), maxRange, pointsNumber);
  checkCudaErrors(cudaGetLastError());
  checkCudaErrors(cudaDeviceSynchronize());
}

} // namespace gpu
