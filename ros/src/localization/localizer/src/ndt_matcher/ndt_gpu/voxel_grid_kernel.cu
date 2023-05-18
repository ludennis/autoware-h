#include <ndt_matcher/ndt_gpu/matrix_device.h>
#include <ndt_matcher/ndt_gpu/symmetric_eigen_solver.h>

namespace gpu
{

__device__ int VoxelId(float x, float y, float z,
  float leafSizeX, float leafSizeY, float leafSizeZ,
  int voxelGridLowerBoundMinX, int voxelGridLowerBoundMinY, int voxelGridLowerBoundMinZ,
  int voxelGridNumX, int voxelGridNumY, int voxelGridNumZ)
  {
    int xIndex =
      static_cast<int>(floorf(x / leafSizeX) -
      static_cast<float>(voxelGridLowerBoundMinX));
    int yIndex =
      static_cast<int>(floorf(y / leafSizeY) -
      static_cast<float>(voxelGridLowerBoundMinY));
    int zIndex =
      static_cast<int>(floorf(z / leafSizeZ) -
      static_cast<float>(voxelGridLowerBoundMinZ));

    return (xIndex + yIndex * voxelGridNumX + zIndex * voxelGridNumX * voxelGridNumY);
  }

__global__ void InitCentroidAndCovariance(float *x, float *y, float *z,
  int *pointsIndicesGroupedByVoxelGrid, int *pointIds,
  double *centroids, double *covariances, int voxelNum)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = idx; i < voxelNum; i += stride)
  {
    MatrixDevice centr(3, 1, voxelNum, centroids + i);
    MatrixDevice cov(3, 3, voxelNum, covariances + i);
    double centr0, centr1, centr2;
    double cov00, cov01, cov02, cov11, cov12, cov22;
    centr0 = centr1 = centr2 = 0.0;
    cov00 = cov11 = cov22 = 1.0;
    cov01 = cov02 = cov12 = 0.0;

    for (int j = pointsIndicesGroupedByVoxelGrid[i];
      j < pointsIndicesGroupedByVoxelGrid[i + 1]; j++)
    {
      int pid = pointIds[j];
      double transformedX = static_cast<double>(x[pid]);
      double transformedY = static_cast<double>(y[pid]);
      double transformedZ = static_cast<double>(z[pid]);

      centr0 += transformedX;
      centr1 += transformedY;
      centr2 += transformedZ;

      cov00 += transformedX * transformedX;
      cov01 += transformedX * transformedY;
      cov02 += transformedX * transformedZ;
      cov11 += transformedY * transformedY;
      cov12 += transformedY * transformedZ;
      cov22 += transformedZ * transformedZ;
    }

    centr(0) = centr0;
    centr(1) = centr1;
    centr(2) = centr2;

    cov(0, 0) = cov00;
    cov(0, 1) = cov01;
    cov(0, 2) = cov02;
    cov(1, 1) = cov11;
    cov(1, 2) = cov12;
    cov(2, 2) = cov22;
  }
}

__global__ void UpdateVoxelCentroid(double *centroid,
  int *pointsNumPerVoxel, int voxelNum)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int vid = index; vid < voxelNum; vid += stride)
  {
    MatrixDevice centr(3, 1, voxelNum, centroid + vid);
    double pointsNum = static_cast<double>(pointsNumPerVoxel[vid]);

    if (pointsNum > 0)
    {
      centr /= pointsNum;
    }
  }
}

__global__ void UpdateVoxelCovariance(double *centroid,
  double *pointsSum, double *covariance, int *pointsNumPerVoxel,
  int voxelNum, int minPointsPerVoxel)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int vid = index; vid < voxelNum; vid += stride)
  {
    MatrixDevice centr(3, 1, voxelNum, centroid + vid);
    MatrixDevice cov(3, 3, voxelNum, covariance + vid);
    MatrixDevice pts(3, 1, voxelNum, pointsSum + vid);
    double pointsNum = static_cast<double>(pointsNumPerVoxel[vid]);

    double c0 = centr(0);
    double c1 = centr(1);
    double c2 = centr(2);
    double p0 = pts(0);
    double p1 = pts(1);
    double p2 = pts(2);

    pointsNumPerVoxel[vid] = (pointsNum < minPointsPerVoxel) ? 0 : pointsNum;

    if (pointsNum >= minPointsPerVoxel)
    {
      double mult = (pointsNum - 1.0) / pointsNum;

      cov(0, 0) = ((cov(0, 0) - 2.0 * p0 * c0) / pointsNum + c0 * c0) * mult;
      cov(0, 1) = ((cov(0, 1) - 2.0 * p0 * c1) / pointsNum + c0 * c1) * mult;
      cov(0, 2) = ((cov(0, 2) - 2.0 * p0 * c2) / pointsNum + c0 * c2) * mult;
      cov(1, 0) = cov(0, 1);
      cov(1, 1) = ((cov(1, 1) - 2.0 * p1 * c1) / pointsNum + c1 * c1) * mult;
      cov(1, 2) = ((cov(1, 2) - 2.0 * p1 * c2) / pointsNum + c1 * c2) * mult;
      cov(2, 0) = cov(0, 2);
      cov(2, 1) = cov(1, 2);
      cov(2, 2) = ((cov(2, 2) - 2.0 * p2 * c2) / pointsNum + c2 * c2) * mult;
    }
  }
}

__global__ void ComputeInverseEigenvectors(double *inverseCovariance,
  int *pointsNumPerVoxel, int voxelNum,
  double *eigenvectors, int minPointsPerVoxel)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int vid = index; vid < voxelNum; vid += stride)
  {
    if (pointsNumPerVoxel[vid] >= minPointsPerVoxel)
    {
      MatrixDevice icov(3, 3, voxelNum, inverseCovariance + vid);
      MatrixDevice eigenVectors(3, 3, voxelNum, eigenvectors + vid);

      eigenVectors.inverse(icov);
    }
    __syncthreads();
  }
}

__global__ void UpdateCovarianceS0(int *pointsNumPerVoxel,
  int voxelNum, double *eigenvalues,
  double *eigenvectors, int minPointsPerVoxel)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int vid = index; vid < voxelNum; vid += stride)
  {
    if (pointsNumPerVoxel[vid] >= minPointsPerVoxel)
    {
      MatrixDevice eigenVectors(3, 3, voxelNum, eigenvectors + vid);

      double eigVal0 = eigenvalues[vid];
      double eigVal1 = eigenvalues[vid + voxelNum];
      double eigVal2 = eigenvalues[vid + 2 * voxelNum];

      eigenVectors(0, 0) *= eigVal0;
      eigenVectors(1, 0) *= eigVal0;
      eigenVectors(2, 0) *= eigVal0;

      eigenVectors(0, 1) *= eigVal1;
      eigenVectors(1, 1) *= eigVal1;
      eigenVectors(2, 1) *= eigVal1;

      eigenVectors(0, 2) *= eigVal2;
      eigenVectors(1, 2) *= eigVal2;
      eigenVectors(2, 2) *= eigVal2;
    }
    __syncthreads();
  }
}

__global__ void UpdateCovarianceS1(double *covariance,
  double *inverseCovariance, int *pointsNumPerVoxel, int voxelNum,
  double *eigenvectors, int minPointsPerVoxel, int col)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int vid = index; vid < voxelNum; vid += stride)
  {
    if (pointsNumPerVoxel[vid] >= minPointsPerVoxel)
    {
      MatrixDevice cov(3, 3, voxelNum, covariance + vid);
      MatrixDevice icov(3, 3, voxelNum, inverseCovariance + vid);
      MatrixDevice eigenVectors(3, 3, voxelNum, eigenvectors + vid);

      double tmp0 = icov(0, col);
      double tmp1 = icov(1, col);
      double tmp2 = icov(2, col);

      cov(0, col) = eigenVectors(0, 0) * tmp0 + eigenVectors(0, 1) * tmp1 +
        eigenVectors(0, 2) * tmp2;
      cov(1, col) = eigenVectors(1, 0) * tmp0 + eigenVectors(1, 1) * tmp1 +
        eigenVectors(1, 2) * tmp2;
      cov(2, col) = eigenVectors(2, 0) * tmp0 + eigenVectors(2, 1) * tmp1 +
        eigenVectors(2, 2) * tmp2;
    }
    __syncthreads();
  }
}

__global__ void ComputeInverseCovariance(double *covariance,
  double *inverseCovariance, int *pointsNumPerVoxel,
  int voxelNum, int minPointsPerVoxel)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int vid = index; vid < voxelNum; vid += stride)
  {
    if (pointsNumPerVoxel[vid] >= minPointsPerVoxel)
    {
      MatrixDevice cov(3, 3, voxelNum, covariance + vid);
      MatrixDevice icov(3, 3, voxelNum, inverseCovariance + vid);

      cov.inverse(icov);
    }
    __syncthreads();
  }
}

__global__ void Normalize(SymmetricEigensolver3x3 sv,
  int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel)
{
  for (int id = threadIdx.x + blockIdx.x * blockDim.x;
    id < voxelNum; id += blockDim.x * gridDim.x)
  {
    if (pointsNumPerVoxel[id] >= minPointsPerVoxel) {}
    sv.NormalizeInput(id);
    __syncthreads();
  }
}

__global__ void ComputeEigenvalues(SymmetricEigensolver3x3 sv,
  int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel)
{
  for (int id = threadIdx.x + blockIdx.x * blockDim.x;
    id < voxelNum; id += blockDim.x * gridDim.x)
  {
    if (pointsNumPerVoxel[id] >= minPointsPerVoxel)
    {
      sv.ComputeEigenvalues(id);
    }
    __syncthreads();
  }
}

__global__ void ComputeEvec00(SymmetricEigensolver3x3 sv,
  int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel)
{
  for (int id = threadIdx.x + blockIdx.x * blockDim.x;
    id < voxelNum; id += blockDim.x * gridDim.x)
  {
    if (pointsNumPerVoxel[id] >= minPointsPerVoxel)
    {
      sv.ComputeEigenvector00(id);
    }
    __syncthreads();
  }
}

__global__ void ComputeEvec01(SymmetricEigensolver3x3 sv,
  int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel)
{
  for (int id = threadIdx.x + blockIdx.x * blockDim.x;
    id < voxelNum; id += blockDim.x * gridDim.x)
  {
    if (pointsNumPerVoxel[id] >= minPointsPerVoxel)
    {
      sv.ComputeEigenvector01(id);
    }
    __syncthreads();
  }
}

__global__ void ComputeEvec10(SymmetricEigensolver3x3 sv,
  int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel)
{
  for (int id = threadIdx.x + blockIdx.x * blockDim.x;
    id < voxelNum; id += blockDim.x * gridDim.x)
  {
    if (pointsNumPerVoxel[id] >= minPointsPerVoxel)
    {
      sv.ComputeEigenvector10(id);
    }
    __syncthreads();
  }
}

__global__ void ComputeEvec11(SymmetricEigensolver3x3 sv,
  int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel)
{
  for (int id = threadIdx.x + blockIdx.x * blockDim.x;
    id < voxelNum; id += blockDim.x * gridDim.x)
  {
    if (pointsNumPerVoxel[id] >= minPointsPerVoxel)
    {
      sv.ComputeEigenvector11(id);
    }
    __syncthreads();
  }
}

__global__ void ComputeEvec2(SymmetricEigensolver3x3 sv,
  int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel)
{
  for (int id = threadIdx.x + blockIdx.x * blockDim.x;
    id < voxelNum; id += blockDim.x * gridDim.x)
  {
    if (pointsNumPerVoxel[id] >= minPointsPerVoxel)
    {
      sv.ComputeEigenvector2(id);
    }
    __syncthreads();
  }
}

__global__ void UpdateEval(SymmetricEigensolver3x3 sv,
  int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel)
{
  for (int id = threadIdx.x + blockIdx.x * blockDim.x;
    id < voxelNum; id += blockDim.x * gridDim.x)
  {
    if (pointsNumPerVoxel[id] >= minPointsPerVoxel)
    {
      sv.UpdateEigenvalues(id);
    }
    __syncthreads();
  }
}

__global__ void UpdateEval2(double *eigenvalues,
  int *pointsNumPerVoxel, int voxelNum, int minPointsPerVoxel)
{
  for (int id = threadIdx.x + blockIdx.x * blockDim.x;
    id < voxelNum; id += blockDim.x * gridDim.x)
  {
    if (pointsNumPerVoxel[id] >= minPointsPerVoxel)
    {
      MatrixDevice eigenValue(3, 1, voxelNum, eigenvalues + id);
      double ev0 = eigenValue(0);
      double ev1 = eigenValue(1);
      double ev2 = eigenValue(2);

      if (ev0 < 0 || ev1 < 0 || ev2 <= 0)
      {
        pointsNumPerVoxel[id] = 0;
        continue;
      }

      double minCovEigenValue = ev2 * 0.01;
      if (ev0 < minCovEigenValue)
      {
        ev0 = minCovEigenValue;
        if (ev1 < minCovEigenValue)
        {
          ev1 = minCovEigenValue;
        }
      }

      eigenValue(0) = ev0;
      eigenValue(1) = ev1;
      eigenValue(2) = ev2;

      __syncthreads();
    }
  }
}

__global__ void FindMax(float *x, float *y, float *z,
  int fullSize, int halfSize)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < halfSize; i += stride)
  {
    x[i] = (i + halfSize < fullSize) ?
      ((x[i] >= x[i + halfSize]) ? x[i] : x[i + halfSize]) : x[i];
    y[i] = (i + halfSize < fullSize) ?
      ((y[i] >= y[i + halfSize]) ? y[i] : y[i + halfSize]) : y[i];
    z[i] = (i + halfSize < fullSize) ?
      ((z[i] >= z[i + halfSize]) ? z[i] : z[i + halfSize]) : z[i];
  }
}

__global__ void FindMin(float *x, float *y, float *z,
  int fullSize, int halfSize)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < halfSize; i += stride)
  {
    x[i] = (i + halfSize < fullSize) ?
      ((x[i] <= x[i + halfSize]) ? x[i] : x[i + halfSize]) : x[i];
    y[i] = (i + halfSize < fullSize) ?
      ((y[i] <= y[i + halfSize]) ? y[i] : y[i + halfSize]) : y[i];
    z[i] = (i + halfSize < fullSize) ?
      ((z[i] <= z[i + halfSize]) ? z[i] : z[i + halfSize]) : z[i];
  }
}

__global__ void FindBoundariesOfCandidateVoxels(float *x,
  float *y, float *z, float radius, int pointsNum,
  float leafSizeX, float leafSizeY, float leafSizeZ,
  int voxelGridUpperBoundMaxX, int voxelGridUpperBoundMaxY, int voxelGridUpperBoundMaxZ,
  int voxelGridLowerBoundMinX, int voxelGridLowerBoundMinY, int voxelGridLowerBoundMinZ,
  int *maxVoxelIndexX, int *maxVoxelIndexY, int *maxVoxelIndexZ,
  int *minVoxelIndexX, int *minVoxelIndexY, int *minVoxelIndexZ,
  int *candidateVoxelNumPerPoint)
{
  int id = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = id; i < pointsNum; i += stride)
  {
    float transformedX = x[i];
    float transformedY = y[i];
    float transformedZ = z[i];

    int maxIdX = static_cast<int>(floorf((transformedX + radius) / leafSizeX));
    int maxIdY = static_cast<int>(floorf((transformedY + radius) / leafSizeY));
    int maxIdZ = static_cast<int>(floorf((transformedZ + radius) / leafSizeZ));

    int minIdX = static_cast<int>(floorf((transformedX - radius) / leafSizeX));
    int minIdY = static_cast<int>(floorf((transformedY - radius) / leafSizeY));
    int minIdZ = static_cast<int>(floorf((transformedZ - radius) / leafSizeZ));

    maxIdX = (maxIdX > voxelGridUpperBoundMaxX) ?
      voxelGridUpperBoundMaxX - voxelGridLowerBoundMinX : maxIdX - voxelGridLowerBoundMinX;
    maxIdY = (maxIdY > voxelGridUpperBoundMaxY) ?
      voxelGridUpperBoundMaxY - voxelGridLowerBoundMinY : maxIdY - voxelGridLowerBoundMinY;
    maxIdZ = (maxIdZ > voxelGridUpperBoundMaxZ) ?
      voxelGridUpperBoundMaxZ - voxelGridLowerBoundMinZ : maxIdZ - voxelGridLowerBoundMinZ;

    minIdX = (minIdX < voxelGridLowerBoundMinX) ?
      0 : minIdX - voxelGridLowerBoundMinX;
    minIdY = (minIdY < voxelGridLowerBoundMinY) ?
      0 : minIdY - voxelGridLowerBoundMinY;
    minIdZ = (minIdZ < voxelGridLowerBoundMinZ) ?
      0 : minIdZ - voxelGridLowerBoundMinZ;

    int vx = maxIdX - minIdX + 1;
    int vy = maxIdY - minIdY + 1;
    int vz = maxIdZ - minIdZ + 1;

    candidateVoxelNumPerPoint[i] = (vx > 0 && vy > 0 && vz > 0) ? vx * vy * vz : 0;

    maxVoxelIndexX[i] = maxIdX;
    maxVoxelIndexY[i] = maxIdY;
    maxVoxelIndexZ[i] = maxIdZ;

    minVoxelIndexX[i] = minIdX;
    minVoxelIndexY[i] = minIdY;
    minVoxelIndexZ[i] = minIdZ;
  }
}

__global__ void CollectValidPoints(int *validPointsMark,
  int *validPointsId, int *validPointsLocation, int pointsNum)
{
  for (int index = threadIdx.x + blockIdx.x * blockDim.x; index < pointsNum;
    index += blockDim.x * gridDim.x)
  {
    if (validPointsMark[index] != 0)
    {
      validPointsId[validPointsLocation[index]] = index;
    }
  }
}

__global__ void UpdateCandidateVoxelIds(int pointsNum,
  int voxelGridNumberX, int voxelGridNumberY, int voxelGridNumberZ,
  int *maxVoxelIndexX, int *maxVoxelIndexY, int *maxVoxelIndexZ,
  int *minVoxelIndexX, int *minVoxelIndexY, int *minVoxelIndexZ,
  int *voxelIdFromVoxelNumPerPoint, int *candidateVoxelId)
{
  int id = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = id; i < pointsNum; i += stride)
  {
    int maxIdX = maxVoxelIndexX[i];
    int maxIdY = maxVoxelIndexY[i];
    int maxIdZ = maxVoxelIndexZ[i];

    int minIdX = minVoxelIndexX[i];
    int minIdY = minVoxelIndexY[i];
    int minIdZ = minVoxelIndexZ[i];

    int writeLocation = voxelIdFromVoxelNumPerPoint[i];

    for (int j = minIdX; j <= maxIdX; j++)
    {
      for (int k = minIdY; k <= maxIdY; k++)
      {
        for (int l = minIdZ; l <= maxIdZ; l++)
        {
          candidateVoxelId[writeLocation] =
            j + k * voxelGridNumberX + l * voxelGridNumberX * voxelGridNumberY;
          writeLocation++;
        }
      }
    }
  }
}

__global__ void InspectCandidateVoxels(float *x, float *y, float *z,
  float radius, int maxNn, int pointsNum,
  double *centroid, int *pointsNumPerVoxel, int offset,
  int *voxelIdFromVoxelNumPerPoint, int *candidateVoxelId,
  int *validVoxelMark, int *validVoxelCount, int *validPointsMark)
{
  int id = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = id; i < pointsNum; i += stride)
  {
    double transformedX = static_cast<double>(x[i]);
    double transformedY = static_cast<double>(y[i]);
    double transformedZ = static_cast<double>(z[i]);

    int nn = 0;
    for (int j = voxelIdFromVoxelNumPerPoint[i];
      j < voxelIdFromVoxelNumPerPoint[i + 1] && nn <= maxNn; j++)
    {
      int pointsNumInVoxel = pointsNumPerVoxel[candidateVoxelId[j]];
      MatrixDevice centr(3, 1, offset, centroid + candidateVoxelId[j]);

      double centroidX = (pointsNumInVoxel > 0) ?
        (transformedX - centr(0)) : radius + 1;
      double centroidY = (pointsNumInVoxel > 0) ?
        (transformedY - centr(1)) : 0;
      double centroidZ = (pointsNumInVoxel > 0) ?
        (transformedZ - centr(2)) : 0;

      bool res = (norm3d(centroidX, centroidY, centroidZ) <= radius);

      validVoxelMark[j] = (res) ? 1 : 0;
      nn += (res) ? 1 : 0;
    }
    validVoxelCount[i] = nn;
    validPointsMark[i] = (nn > 0) ? 1 : 0;

    __syncthreads();
  }
}

__global__ void CollectValidVoxels(int *validVoxelsMark,
  int *candidateVoxelId, int *output, int *writingLocation, int candidateVoxelNum)
{
  for (int index = threadIdx.x + blockIdx.x * blockDim.x;
    index < candidateVoxelNum; index += blockDim.x * gridDim.x)
  {
    if (validVoxelsMark[index] == 1)
    {
      output[writingLocation[index]] = candidateVoxelId[index];
    }
  }
}

__global__ void CollectValidVoxelCount(int *inputValidVoxelCount,
  int *outputValidVoxelCount, int *writingLocation, int pointsNum)
{
  for (int id = threadIdx.x + blockIdx.x * blockDim.x;
    id < pointsNum; id += blockDim.x * gridDim.x)
  {
    if (inputValidVoxelCount[id] != 0)
    {
      outputValidVoxelCount[writingLocation[id]] =
        inputValidVoxelCount[id];
    }
  }
}

__global__ void BuildOctreeParents(
  int childGridX, int childGridY, int childGridZ,
  double *parentCentroids, int parentGridX, int parentGridY, int parentGridZ,
  double minX, double minY, double minZ,
  double leafSizeX, double leafSizeY, double leafSizeZ, int layer)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;
  int idy = threadIdx.y + blockIdx.y * blockDim.y;
  int idz = threadIdx.z + blockIdx.z * blockDim.z;

  if (idx < parentGridX && idy < parentGridY && idz < parentGridZ)
  {
    int parentIdx = idx + idy * parentGridX + idz * parentGridX * parentGridY;
    MatrixDevice parentCentr(3, 1,
      parentGridX * parentGridY * parentGridZ, parentCentroids + parentIdx);
    double pc0, pc1, pc2;
    int count = 0;

    pc0 = 0.0;
    pc1 = 0.0;
    pc2 = 0.0;

    if (idx * 2 + 1  >= childGridX)
    {
      childGridX += 1;
    }
    if (idy * 2 + 1  >= childGridY)
    {
      childGridY += 1;
    }
    if (idz * 2 + 1  >= childGridZ)
    {
      childGridZ += 1;
    }


    for (int i = idx * 2; i < idx * 2 + 2 && i < childGridX; i++)
    {
      for (int j = idy * 2; j < idy * 2 + 2 && j < childGridY; j++)
      {
        for (int k = idz * 2; k < idz * 2 + 2 && k < childGridZ; k++)
        {
          pc0 += floor(minX) + pow(2, layer) * leafSizeX * 0.5 * (2 * i + 1);
          pc1 += floor(minY) + pow(2, layer) * leafSizeY * 0.5 * (2 * j + 1);
          pc2 += floor(minZ) + pow(2, layer) * leafSizeZ * 0.5 * (2 * k + 1);
          count++;

          __syncthreads();
        }
      }
    }

    parentCentr(0) = pc0 / count;
    parentCentr(1) = pc1 / count;
    parentCentr(2) = pc2 / count;
  }
}

__global__ void InsertPointsToGrid(float *x, float *y, float *z,
  int pointsNum, int *pointsNumPerVoxel, int voxelNum,
  int voxelGridNumX, int voxelGridNumY, int voxelGridNumZ,
  float leafSizeX, float leafSizeY, float leafSizeZ,
  int voxelGridLowerBoundMinX, int voxelGridLowerBoundMinY, int voxelGridLowerBoundMinZ)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < pointsNum; i += stride)
  {
    float transformedX = x[i];
    float transformedY = y[i];
    float transformedZ = z[i];
    int voxelGridId = VoxelId(transformedX, transformedY, transformedZ,
      leafSizeX, leafSizeY, leafSizeZ,
      voxelGridLowerBoundMinX, voxelGridLowerBoundMinY, voxelGridLowerBoundMinZ,
      voxelGridNumX, voxelGridNumY, voxelGridNumZ);

    int ptrIncrement = (voxelGridId < voxelNum) * voxelGridId;
    int incrementalValue = (voxelGridId < voxelNum);
    atomicAdd(pointsNumPerVoxel + ptrIncrement, incrementalValue);
  }
}

__global__ void ScatterPointsToVoxels(float *x, float *y, float *z,
  int pointsNum, int voxelNum, float leafSizeX, float leafSizeY, float leafSizeZ,
  int voxelGridLowerBoundMinX, int voxelGridLowerBoundMinY, int voxelGridLowerBoundMinZ,
  int voxelGridNumX, int voxelGridNumY, int voxelGridNumZ,
  int *writingLocations, int *pointIds)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = idx; i < pointsNum; i += stride)
  {
    int voxelGridId = VoxelId(x[i], y[i], z[i],
      leafSizeX, leafSizeY, leafSizeZ,
      voxelGridLowerBoundMinX, voxelGridLowerBoundMinY, voxelGridLowerBoundMinZ,
      voxelGridNumX, voxelGridNumY, voxelGridNumZ);

    int ptrIncrement = (voxelGridId < voxelNum) * voxelGridId;
    int incrementalValue = (voxelGridId < voxelNum);
    int loc =  atomicAdd(writingLocations + ptrIncrement, incrementalValue);

    pointIds[loc] = i;
  }
}

__global__ void NearestOctreeNodeSearch(
  float *x, float *y, float *z,
  int *voxelIndexX, int *voxelIndexY, int *voxelIndexZ, int pointsNum,
  double *centroids, int voxelGridNumX, int voxelGridNumY, int voxelGridNumZ,
  int nodeNum, int layer)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = idx; i < pointsNum; i += stride)
  {
    int vx = voxelIndexX[i];
    int vy = voxelIndexY[i];
    int vz = voxelIndexZ[i];
    double minDist = DBL_MAX;
    double transformedX = static_cast<double>(x[i]);
    double transformedY = static_cast<double>(y[i]);
    double transformedZ = static_cast<double>(z[i]);
    double currentDist;
    int outputIdX, outputIdY, outputIdZ;

    outputIdX = vx;
    outputIdY = vy;
    outputIdZ = vz;

    double tmpPointX, tmpPointY, tmpPointZ;
    for (int j = vx * 2; j < vx * 2 + 2 && j < voxelGridNumX; j++)
    {
      for (int k = vy * 2; k < vy * 2 + 2 && k < voxelGridNumY; k++)
      {
        for (int l = vz * 2; l < vz * 2 + 2 && l < voxelGridNumZ; l++)
        {
          int nodeId = j + k * voxelGridNumX + l * voxelGridNumX * voxelGridNumY;
          MatrixDevice nodeCentr(3, 1, nodeNum, centroids + nodeId);

          tmpPointX = nodeCentr(0) - transformedX;
          tmpPointY = nodeCentr(1) - transformedY;
          tmpPointZ = nodeCentr(2) - transformedZ;

          currentDist = norm3d(tmpPointX, tmpPointY, tmpPointZ);
          bool res = (currentDist < minDist);

          outputIdX = (res) ? j : outputIdX;
          outputIdY = (res) ? k : outputIdY;
          outputIdZ = (res) ? l : outputIdZ;

          minDist = (res) ? currentDist : minDist;
        }
      }
    }
    voxelIndexX[i] = outputIdX;
    voxelIndexY[i] = outputIdY;
    voxelIndexZ[i] = outputIdZ;
  }
}

__global__ void NearestPointSearch(float *queryPointsX,
  float *queryPointsY, float *queryPointsZ, int queryPointsNum,
  float *referencePointsX, float *referencePointsY, float *referencePointsZ,
  int *voxelIdX, int *voxelIdY, int *voxelIdZ,
  int voxelGridNumX, int voxelGridNumY, int voxelGridNumZ,
  int *pointIndexFromVoxel, int *pointId, double *minDistance)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = idx; i < queryPointsNum; i += stride)
  {
    int voxelId = voxelIdX[i] + voxelIdY[i] * voxelGridNumX +
      voxelIdZ[i] * voxelGridNumX * voxelGridNumY;
    float corQueryPointX = queryPointsX[i];
    float corQueryPointY = queryPointsY[i];
    float corQueryPointZ = queryPointsZ[i];
    float minDist = FLT_MAX;

    for (int j = pointIndexFromVoxel[voxelId];
      j < pointIndexFromVoxel[voxelId + 1]; j++)
    {
      int pid = pointId[j];
      float corReferenceX = referencePointsX[pid];
      float corReferenceY = referencePointsY[pid];
      float corReferenceZ = referencePointsZ[pid];
      corReferenceX -= corQueryPointX;
      corReferenceY -= corQueryPointY;
      corReferenceZ -= corQueryPointZ;

      minDist = fminf(norm3df(corReferenceX, corReferenceY, corReferenceZ), minDist);
    }
    minDistance[i] = static_cast<double>(minDist);
  }
}

__global__ void VerifyDistances(int *validDistance,
  double *minDistance, double maxRange, int pointsNum)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = idx; i < pointsNum; i += stride)
  {
    bool check = (minDistance[i] < maxRange);
    validDistance[i] = (check) ? 1 : 0;

    if (!check)
    {
      minDistance[i] = 0;
    }
  }
}

} // namespace gpu
