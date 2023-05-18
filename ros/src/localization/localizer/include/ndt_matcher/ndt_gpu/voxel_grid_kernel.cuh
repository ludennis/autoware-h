#ifndef VOXEL_GRID_KERNEL_H_
#define VOXEL_GRID_KERNEL_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <float.h>
#include <memory>
#include <ndt_matcher/ndt_gpu/common.h>
#include <ndt_matcher/ndt_gpu/debug.h>
#include <ndt_matcher/ndt_gpu/matrix_device.h>
#include <ndt_matcher/ndt_gpu/matrix_host.h>
#include <ndt_matcher/ndt_gpu/memory.h>
#include <ndt_matcher/ndt_gpu/symmetric_eigen_solver.h>

namespace gpu
{

  __device__ int VoxelId(float x, float y, float z,
    float leafSizeX, float leafSizeY, float leafSizeZ,
    int voxelGridLowerBoundMinX, int voxelGridLowerBoundMinY, int voxelGridLowerBoundMinZ,
    int voxelGridNumX, int voxelGridNumY, int voxelGridNumZ);

  __global__ void InitCentroidAndCovariance(float *x, float *y, float *z,
    int *pointsIndicesGroupedByVoxelGrid, int *pointIds,
    double *centroids, double *covariances, int voxelNum);

  __global__ void UpdateVoxelCentroid(double *centroid,
    int *pointsNumPerVoxel, int voxelNum);

  __global__ void UpdateVoxelCovariance(double *centroid,
    double *pointsSum, double *covariance, int *pointsNumPerVoxel,
    int voxelNum, int minPointsPerVoxel);

  __global__ void ComputeInverseEigenvectors(double *inverseCovariance,
    int *pointsNumPerVoxel, int voxelNum,
    double *eigenvectors, int minPointsPerVoxel);

  __global__ void UpdateCovarianceS0(int *pointsNumPerVoxel,
    int voxelNum, double *eigenvalues,
    double *eigenvectors, int minPointsPerVoxel);

  __global__ void UpdateCovarianceS1(double *covariance,
    double *inverseCovariance, int *pointsNumPerVoxel, int voxelNum,
    double *eigenvectors, int minPointsPerVoxel, int col);

  __global__ void ComputeInverseCovariance(double *covariance,
    double *inverseCovariance, int *pointsNumPerVoxel,
    int voxelNum, int minPointsPerVoxel);

  __global__ void Normalize(SymmetricEigensolver3x3 sv,
    int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel);

  __global__ void ComputeEigenvalues(SymmetricEigensolver3x3 sv,
    int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel);

  __global__ void ComputeEvec00(SymmetricEigensolver3x3 sv,
    int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel);

  __global__ void ComputeEvec01(SymmetricEigensolver3x3 sv,
    int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel);

  __global__ void ComputeEvec10(SymmetricEigensolver3x3 sv,
    int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel);

  __global__ void ComputeEvec11(SymmetricEigensolver3x3 sv,
    int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel);

  __global__ void ComputeEvec2(SymmetricEigensolver3x3 sv,
    int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel);

  __global__ void UpdateEval(SymmetricEigensolver3x3 sv,
    int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel);

  __global__ void UpdateEval2(double *eigenvalues,
    int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel);

  __global__ void FindMax(float *x, float *y, float *z,
    int fullSize, int halfSize);

  __global__ void FindMin(float *x, float *y, float *z,
    int fullSize, int halfSize);

  __global__ void FindBoundariesOfCandidateVoxels(float *x,
    float *y, float *z, float radius, int pointsNum,
    float leafSizeX, float leafSizeY, float leafSizeZ,
    int voxelGridUpperBoundMaxX, int voxelGridUpperBoundMaxY, int voxelGridUpperBoundMaxZ,
    int voxelGridLowerBoundMinX, int voxelGridLowerBoundMinY, int voxelGridLowerBoundMinZ,
    int *maxVoxelIndexX, int *maxVoxelIndexY, int *maxVoxelIndexZ,
    int *minVoxelIndexX, int *minVoxelIndexY, int *minVoxelIndexZ,
    int *candidateVoxelNumPerPoint);

  __global__ void CollectValidPoints(int *validPointsMark,
    int *validPointsId, int *validPointsLocation, int pointsNum);

  __global__ void UpdateCandidateVoxelIds(int pointsNum,
    int voxelGridNumberX, int voxelGridNumberY, int voxelGridNumberZ,
    int *maxVoxelIndexX, int *maxVoxelIndexY, int *maxVoxelIndexZ,
    int *minVoxelIndexX, int *minVoxelIndexY, int *minVoxelIndexZ,
    int *voxelIdFromVoxelNumPerPoint, int *candidateVoxelId);

  __global__ void InspectCandidateVoxels(float *x, float *y, float *z,
    float radius, int maxNn, int pointsNum,
    double *centroid, int *pointsNumPerVoxel, int offset,
    int *voxelIdFromVoxelNumPerPoint, int *candidateVoxelId,
    int *validVoxelMark, int *validVoxelCount, int *validPointsMark);

  __global__ void CollectValidVoxels(int *validVoxelsMark,
    int *candidateVoxelId, int *output,
    int *writingLocation, int candidateVoxelNum);

  __global__ void CollectValidVoxelCount(int *inputValidVoxelCount,
    int *outputValidVoxelCount, int *writingLocation, int pointsNum);

  __global__ void BuildOctreeParents(
    int childGridX, int childGridY, int childGridZ,
    double *parentCentroids, int parentGridX, int parentGridY, int parentGridZ,
    double minX, double minY, double minZ,
    double leafSizeX, double leafSizeY, double leafSizeZ, int layer);

  __global__ void InsertPointsToGrid(float *x, float *y, float *z,
    int pointsNum, int *pointsNumPerVoxel, int voxelNum,
    int voxelGridNumX, int voxelGridNumY, int voxelGridNumZ,
    float leafSizeX, float leafSizeY, float leafSizeZ,
    int voxelGridLowerBoundMinX, int voxelGridLowerBoundMinY, int voxelGridLowerBoundMinZ);

  __global__ void ScatterPointsToVoxels(float *x, float *y, float *z,
    int pointsNum, int voxelNum, float leafSizeX, float leafSizeY, float leafSizeZ,
    int voxelGridLowerBoundMinX, int voxelGridLowerBoundMinY, int voxelGridLowerBoundMinZ,
    int voxelGridNumX, int voxelGridNumY, int voxelGridNumZ,
    int *writingLocations, int *pointIds);

  __global__ void NearestOctreeNodeSearch(
    float *x, float *y, float *z,
    int *voxelIndexX, int *voxelIndexY, int *voxelIndexZ, int pointsNum,
    double *centroids, int voxelGridNumX, int voxelGridNumY, int voxelGridNumZ,
    int nodeNum, int layer);

  __global__ void NearestPointSearch(float *queryPointsX,
    float *queryPointsY, float *queryPointsZ, int queryPointsNum,
    float *referencePointsX, float *referencePointsY, float *referencePointsZ,
    int *voxelIdX, int *voxelIdY, int *voxelIdZ,
    int voxelGridNumX, int voxelGridNumY, int voxelGridNumZ,
    int *pointIndexFromVoxel, int *pointId, double *minDistance);

  __global__ void VerifyDistances(int *validDistance,
    double *minDistance, double maxRange, int pointsNum);

} // namespace gpu

#endif //VOXEL_GRID_KERNEL_H_
