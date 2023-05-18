#include <ndt_matcher/ndt_gpu/matrix_device.h>

namespace gpu
{

  __global__ void ComputePointGradients0(float *x, float *y, float *z,
    int *validPoints, int validPointsNum,
    double *djAng, double *pg00, double *pg11, double *pg22,
    double *pg13, double *pg23, double *pg04, double *pg14)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    __shared__ double jAng[12];

    if (threadIdx.x < 12)
    {
      jAng[threadIdx.x] = djAng[threadIdx.x];
    }

    __syncthreads();

    for (int i = id; i < validPointsNum; i += stride)
    {
      int pid = validPoints[i];

      double originalCoordPointX = static_cast<double>(x[pid]);
      double originalCoordPointY = static_cast<double>(y[pid]);
      double originalCoordPointZ = static_cast<double>(z[pid]);

      pg00[i] = 1;
      pg11[i] = 1;
      pg22[i] = 1;

      pg13[i] = originalCoordPointX * jAng[0] +
        originalCoordPointY * jAng[1] + originalCoordPointZ * jAng[2];
      pg23[i] = originalCoordPointX * jAng[3] +
        originalCoordPointY * jAng[4] + originalCoordPointZ * jAng[5];
      pg04[i] = originalCoordPointX * jAng[6] +
        originalCoordPointY * jAng[7] + originalCoordPointZ * jAng[8];
      pg14[i] = originalCoordPointX * jAng[9] +
        originalCoordPointY * jAng[10] + originalCoordPointZ * jAng[11];
    }
  }

  __global__ void ComputePointGradients1(float *x, float *y, float *z,
    int *validPoints, int validPointsNum, double *djAng,
    double *pg24, double *pg05, double *pg15, double *pg25)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    __shared__ double jAng[12];

    if (threadIdx.x < 12)
    {
      jAng[threadIdx.x] = djAng[threadIdx.x + 12];
    }

    __syncthreads();

    for (int i = id; i < validPointsNum; i += stride)
    {
      int pid = validPoints[i];

      double originalCoordPointX = static_cast<double>(x[pid]);
      double originalCoordPointY = static_cast<double>(y[pid]);
      double originalCoordPointZ = static_cast<double>(z[pid]);

      pg24[i] = originalCoordPointX * jAng[0] +
        originalCoordPointY * jAng[1] + originalCoordPointZ * jAng[2];
      pg05[i] = originalCoordPointX * jAng[3] +
        originalCoordPointY * jAng[4] + originalCoordPointZ * jAng[5];
      pg15[i] = originalCoordPointX * jAng[6] +
        originalCoordPointY * jAng[7] + originalCoordPointZ * jAng[8];
      pg25[i] = originalCoordPointX * jAng[9] +
        originalCoordPointY * jAng[10] + originalCoordPointZ * jAng[11];
    }
  }

  __global__ void ComputePointHessian0(float *x, float *y, float *z,
    int *validPoints, int validPointsNum, double *dhAng,
    double *ph93, double *ph103, double *ph113,
    double *ph123, double *ph94, double *ph133,
    double *ph104, double *ph143, double *ph114,
    double *ph153, double *ph95, double *ph163,
    double *ph105, double *ph173, double *ph115)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    __shared__ double hAng[18];

    if (threadIdx.x < 18)
    {
      hAng[threadIdx.x] = dhAng[threadIdx.x];
    }

    __syncthreads();

    for (int i = id; i < validPointsNum; i += stride)
    {
      int pid = validPoints[i];

      double originalCoordPointX = static_cast<double>(x[pid]);
      double originalCoordPointY = static_cast<double>(y[pid]);
      double originalCoordPointZ = static_cast<double>(z[pid]);

      ph93[i] = 0;
      ph103[i] = originalCoordPointX * hAng[0] + originalCoordPointY * hAng[1] +
        originalCoordPointZ * hAng[2];
      ph113[i] = originalCoordPointX * hAng[3] + originalCoordPointY * hAng[4] +
        originalCoordPointZ * hAng[5];

      ph123[i] = ph94[i] = 0;
      ph133[i] = ph104[i] = originalCoordPointX * hAng[6] +
        originalCoordPointY * hAng[7] + originalCoordPointZ * hAng[8];
      ph143[i] = ph114[i] = originalCoordPointX * hAng[9] +
        originalCoordPointY * hAng[10] + originalCoordPointZ * hAng[11];

      ph153[i] = ph95[i] = 0;
      ph163[i] = ph105[i] = originalCoordPointX * hAng[12] +
        originalCoordPointY * hAng[13] + originalCoordPointZ * hAng[14];
      ph173[i] = ph115[i] = originalCoordPointX * hAng[15] +
        originalCoordPointY * hAng[16] + originalCoordPointZ * hAng[17];
    }
  }

  __global__ void ComputePointHessian1(float *x, float *y, float *z,
    int *validPoints, int validPointsNum, double *dhAng,
    double *ph124, double *ph134, double *ph144,
    double *ph154, double *ph125, double *ph164,
    double *ph135, double *ph174, double *ph145)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    __shared__ double hAng[18];

    if (threadIdx.x < 18)
    {
      hAng[threadIdx.x] = dhAng[18 + threadIdx.x];
    }

    __syncthreads();

    for (int i = id; i < validPointsNum; i += stride)
    {
      int pid = validPoints[i];

      double originalCoordPointX = static_cast<double>(x[pid]);
      double originalCoordPointY = static_cast<double>(y[pid]);
      double originalCoordPointZ = static_cast<double>(z[pid]);

      ph124[i] = originalCoordPointX * hAng[0] + originalCoordPointY * hAng[1] +
        originalCoordPointZ * hAng[2];
      ph134[i] = originalCoordPointX * hAng[3] + originalCoordPointY * hAng[4] +
        originalCoordPointZ * hAng[5];
      ph144[i] = originalCoordPointX * hAng[6] + originalCoordPointY * hAng[7] +
        originalCoordPointZ * hAng[8];

      ph154[i] = ph125[i] = originalCoordPointX * hAng[9] +
        originalCoordPointY * hAng[10] + originalCoordPointZ * hAng[11];
      ph164[i] = ph135[i] = originalCoordPointX * hAng[12] +
        originalCoordPointY * hAng[13] + originalCoordPointZ * hAng[14];
      ph174[i] = ph145[i] = originalCoordPointX * hAng[15] +
        originalCoordPointY * hAng[16] + originalCoordPointZ * hAng[17];
    }
  }

  __global__ void ComputePointHessian2(float *x, float *y, float *z,
    int *validPoints, int validPointsNum, double *dhAng,
    double *ph155, double *ph165, double *ph175)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    __shared__ double hAng[9];

    if (threadIdx.x < 9)
    {
      hAng[threadIdx.x] = dhAng[36 + threadIdx.x];
    }

    __syncthreads();

    for (int i = id; i < validPointsNum; i += stride)
    {
      int pid = validPoints[i];

      double originalCoordPointX = static_cast<double>(x[pid]);
      double originalCoordPointY = static_cast<double>(y[pid]);
      double originalCoordPointZ = static_cast<double>(z[pid]);

      ph155[i] = originalCoordPointX * hAng[0] + originalCoordPointY * hAng[1] +
        originalCoordPointZ * hAng[2];
      ph165[i] = originalCoordPointX * hAng[3] + originalCoordPointY * hAng[4] +
        originalCoordPointZ * hAng[5];
      ph175[i] = originalCoordPointX * hAng[6] + originalCoordPointY * hAng[7] +
        originalCoordPointZ * hAng[8];
    }
  }

  __global__ void ComputeScoreList(int *voxelIdFromVoxelNumPerPoint,
    int validPointsNum, double *eXCovX, double gaussD1, double *score)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;

    for (int i = id; i < validPointsNum; i += stride)
    {
      double scoreInc = 0;
      for (int vid = voxelIdFromVoxelNumPerPoint[i];
        vid < voxelIdFromVoxelNumPerPoint[i + 1]; vid++)
      {
        double tmpEx = eXCovX[vid];
        scoreInc += (tmpEx > 1 || tmpEx < 0 ||
          tmpEx != tmpEx) ? 0 : -gaussD1 * tmpEx;
      }
      score[i] = scoreInc;
    }
  }

  __global__ void ComputeScoreGradientList(
    float *transformedX, float *transformedY, float *transformedZ,
    int *validPoints, int *voxelIdFromVoxelNumPerPoint,
    int *voxelId, int validPointsNum,
    double *centroidX, double *centroidY, double *centroidZ,
    double *eXCovX, double *covDxdPi, double gaussD1, int validVoxelNum,
    double *scoreGradients)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    int col = blockIdx.y;

    if (col < 6)
    {
      double *sg = scoreGradients + col * validPointsNum;
      double *covDxdPiMat0 = covDxdPi + col * validVoxelNum;
      double *covDxdPiMat1 = covDxdPiMat0 + 6 * validVoxelNum;
      double *covDxdPiMat2 = covDxdPiMat1 + 6 * validVoxelNum;

      for (int i = id; i < validPointsNum; i += stride)
      {
        int pid = validPoints[i];
        double dX = static_cast<double>(transformedX[pid]);
        double dY = static_cast<double>(transformedY[pid]);
        double dZ = static_cast<double>(transformedZ[pid]);
        double tmpSg = 0.0;

        for ( int j = voxelIdFromVoxelNumPerPoint[i];
          j < voxelIdFromVoxelNumPerPoint[i + 1]; j++)
        {
          int vid = voxelId[j];
          double tmpEx = eXCovX[j];
          if (!(tmpEx > 1 || tmpEx < 0 || tmpEx != tmpEx))
          {
            tmpEx *= gaussD1;
            tmpSg += ((dX - centroidX[vid]) * covDxdPiMat0[j] +
              (dY - centroidY[vid]) * covDxdPiMat1[j] +
              (dZ - centroidZ[vid]) * covDxdPiMat2[j]) * tmpEx;
          }
        }
        sg[i] = tmpSg;
      }
    }
  }

  __global__ void ComputeExCovX(
    float *transformedX, float *transformedY, float *transformedZ, int *validPoints,
    int *voxelIdFromVoxelNumPerPoint, int *voxelId, int validPointsNum,
    double *centrX, double *centrY, double *centrZ,
    double gaussD1, double gaussD2, double *eXCovX,
    double *icov00, double *icov01, double *icov02,
    double *icov10, double *icov11, double *icov12,
    double *icov20, double *icov21, double *icov22)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;

    for (int i = id; i < validPointsNum; i += stride)
    {
      int pid = validPoints[i];
      double dX = static_cast<double>(transformedX[pid]);
      double dY = static_cast<double>(transformedY[pid]);
      double dZ = static_cast<double>(transformedZ[pid]);
      double tX, tY, tZ;

      for ( int j = voxelIdFromVoxelNumPerPoint[i];
        j < voxelIdFromVoxelNumPerPoint[i + 1]; j++)
      {
        int vid = voxelId[j];
        tX = dX - centrX[vid];
        tY = dY - centrY[vid];
        tZ = dZ - centrZ[vid];
        eXCovX[j] =  exp(-gaussD2 * ((tX * icov00[vid] + tY * icov01[vid] +
          tZ * icov02[vid]) * tX + ((tX * icov10[vid] + tY * icov11[vid] +
          tZ * icov12[vid]) * tY) + ((tX * icov20[vid] + tY * icov21[vid] +
          tZ * icov22[vid]) * tZ)) / 2.0);
      }
    }
  }

  __global__ void UpdateExCovX(double *eXCovX, double gaussD2, int validVoxelNum)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    for (int i = id; i < validVoxelNum; i += stride)
    {
      eXCovX[i] *= gaussD2;
    }
  }

  __global__ void ComputeCovDxdPi(int *voxelIdFromVoxelNumPerPoint,
    int *voxelId, int validPointsNum, double *inverseCovariance, int voxelNum,
    double *pointGradients, double *covDxdPi, int validVoxelNum)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    int row = blockIdx.y;
    int col = blockIdx.z;

    if (row < 3 && col < 6)
    {
      double *icov0 = inverseCovariance + row * 3 * voxelNum;
      double *icov1 = icov0 + voxelNum;
      double *icov2 = icov1 + voxelNum;
      double *covDxdPiTmp = covDxdPi + (row * 6 + col) * validVoxelNum;
      double *pgTmp0 = pointGradients + col * validPointsNum;
      double *pgTmp1 = pgTmp0 + 6 * validPointsNum;
      double *pgTmp2 = pgTmp1 + 6 * validPointsNum;

      for (int i = id; i < validPointsNum; i += stride)
      {
        double pg0 = pgTmp0[i];
        double pg1 = pgTmp1[i];
        double pg2 = pgTmp2[i];

        for ( int j = voxelIdFromVoxelNumPerPoint[i];
          j < voxelIdFromVoxelNumPerPoint[i + 1]; j++)
        {
          int vid = voxelId[j];
          covDxdPiTmp[j] = icov0[vid] * pg0 + icov1[vid] * pg1 + icov2[vid] * pg2;
        }
      }
    }
  }

  __global__ void ComputeHessianListS0(
    float *transformedX, float *transformedY, float *transformedZ,
    int *validPoints, int *voxelIdFromVoxelNumPerPoint,
    int *voxelId, int validPointsNum,
    double *centroidX, double *centroidY, double *centroidZ,
    double *icov00, double *icov01, double *icov02,
    double *icov10, double *icov11, double *icov12,
    double *icov20, double *icov21, double *icov22,
    double *pointGradients, double *tmpHessian, int validVoxelNum)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    int col = blockIdx.y;

    if (col < 6)
    {
      double *tmpPg0 = pointGradients + col * validPointsNum;
      double *tmpPg1 = tmpPg0 + 6 * validPointsNum;
      double *tmpPg2 = tmpPg1 + 6 * validPointsNum;
      double *tmpH = tmpHessian + col * validVoxelNum;

      for (int i = id; i < validPointsNum; i += stride)
      {
        int pid = validPoints[i];
        double dX = static_cast<double>(transformedX[pid]);
        double dY = static_cast<double>(transformedY[pid]);
        double dZ = static_cast<double>(transformedZ[pid]);
        double pg0 = tmpPg0[i];
        double pg1 = tmpPg1[i];
        double pg2 = tmpPg2[i];
        for ( int j = voxelIdFromVoxelNumPerPoint[i];
          j < voxelIdFromVoxelNumPerPoint[i + 1]; j++)
        {
          int vid = voxelId[j];
          tmpH[j] = (dX - centroidX[vid]) * (icov00[vid] * pg0 +
            icov01[vid] * pg1 + icov02[vid] * pg2) + (dY - centroidY[vid]) *
            (icov10[vid] * pg0 + icov11[vid] * pg1 + icov12[vid] * pg2) +
            (dZ - centroidZ[vid]) * (icov20[vid] * pg0 + icov21[vid] * pg1 + icov22[vid] * pg2);
        }
      }
    }
  }

  __global__ void ComputeHessianListS1(
    float *transformedX, float *transformedY, float *transformedZ,
    int *validPoints, int *voxelIdFromVoxelNumPerPoint,
    int *voxelId, int validPointsNum,
    double *centroidX, double *centroidY, double *centroidZ,
    double gaussD1, double gaussD2, double *hessians,
    double *eXCovX, double *tmpHessian, double *covDxdPi,
    double *pointGradients, int validVoxelNum)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    int row = blockIdx.y;
    int col = blockIdx.z;

    if (row < 6 && col < 6)
    {
      double *covDxdPiMat0 = covDxdPi + row * validVoxelNum;
      double *covDxdPiMat1 = covDxdPiMat0 + 6 * validVoxelNum;
      double *covDxdPiMat2 = covDxdPiMat1 + 6 * validVoxelNum;
      double *tmpH = tmpHessian + col * validVoxelNum;
      double *h = hessians + (row * 6 + col) * validPointsNum;
      double *tmpPg0 = pointGradients + col * validPointsNum;
      double *tmpPg1 = tmpPg0 + 6 * validPointsNum;
      double *tmpPg2 = tmpPg1 + 6 * validPointsNum;

      for (int i = id; i < validPointsNum; i += stride)
      {
        int pid = validPoints[i];
        double dX = static_cast<double>(transformedX[pid]);
        double dY = static_cast<double>(transformedY[pid]);
        double dZ = static_cast<double>(transformedZ[pid]);
        double pg0 = tmpPg0[i];
        double pg1 = tmpPg1[i];
        double pg2 = tmpPg2[i];
        double finalHessian = 0.0;

        for ( int j = voxelIdFromVoxelNumPerPoint[i];
          j < voxelIdFromVoxelNumPerPoint[i + 1]; j++)
        {
          int vid = voxelId[j];
          double tmpEx = eXCovX[j];

          if (!(tmpEx > 1 || tmpEx < 0 || tmpEx != tmpEx))
          {
            double covDxd0 = covDxdPiMat0[j];
            double covDxd1 = covDxdPiMat1[j];
            double covDxd2 = covDxdPiMat2[j];
            tmpEx *= gaussD1;
            finalHessian += -gaussD2 * ((dX - centroidX[vid]) * covDxd0 +
              (dY - centroidY[vid]) * covDxd1 +
              (dZ - centroidZ[vid]) * covDxd2) * tmpH[j] * tmpEx;
            finalHessian += (pg0 * covDxd0 + pg1 * covDxd1 + pg2 * covDxd2) * tmpEx;
          }
        }
        h[i] = finalHessian;
      }
    }
  }

  __global__ void ComputeHessianListS2(
    float *transformedX, float *transformedY, float *transformedZ,
    int *validPoints, int *voxelIdFromVoxelNumPerPoint,
    int *voxelId, int validPointsNum,
    double *centroidX, double *centroidY, double *centroidZ,
    double gaussD1, double *eXCovX,
    double *icov00, double *icov01, double *icov02,
    double *icov10, double *icov11, double *icov12,
    double *icov20, double *icov21, double *icov22,
    double *pointHessians, double *hessians)
  {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    int row = blockIdx.y;
    int col = blockIdx.z;

    if (row < 6 && col < 6)
    {
      double *h = hessians + (row * 6 + col) * validPointsNum;
      double *tmpPh0 = pointHessians + ((3 * row) * 6 + col) * validPointsNum;
      double *tmpPh1 = tmpPh0 + 6 * validPointsNum;
      double *tmpPh2 = tmpPh1 + 6 * validPointsNum;

      for (int i = id; i < validPointsNum; i += stride)
      {
        int pid = validPoints[i];
        double dX = static_cast<double>(transformedX[pid]);
        double dY = static_cast<double>(transformedY[pid]);
        double dZ = static_cast<double>(transformedZ[pid]);
        double ph0 = tmpPh0[i];
        double ph1 = tmpPh1[i];
        double ph2 = tmpPh2[i];
        double finalHessian = h[i];

        for ( int j = voxelIdFromVoxelNumPerPoint[i];
          j < voxelIdFromVoxelNumPerPoint[i + 1]; j++)
        {
          int vid = voxelId[j];
          double tmpEx = eXCovX[j];

          if (!(tmpEx > 1 || tmpEx < 0 || tmpEx != tmpEx))
          {
            tmpEx *= gaussD1;
            finalHessian += (dX - centroidX[vid]) * (icov00[vid] *
              ph0 + icov01[vid] * ph1 + icov02[vid] * ph2) * tmpEx;
            finalHessian += (dY - centroidY[vid]) * (icov10[vid] *
              ph0 + icov11[vid] * ph1 + icov12[vid] * ph2) * tmpEx;
            finalHessian += (dZ - centroidZ[vid]) * (icov20[vid] *
              ph0 + icov21[vid] * ph1 + icov22[vid] * ph2) * tmpEx;
          }
        }
        h[i] = finalHessian;
      }
    }
  }


  __global__ void MatrixSum(double *matrixList, int fullSize, int halfSize,
    int rows, int cols, int offset)
  {
    int index = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    int row = blockIdx.y;
    int col = blockIdx.z;

    for (int i = index; i < halfSize && row < rows && col < cols; i += stride)
    {
      MatrixDevice left(rows, cols, offset, matrixList + i);
      double *rightPtr = (i + halfSize < fullSize) ?
        matrixList + i + halfSize : NULL;
      MatrixDevice right(rows, cols, offset, rightPtr);

      if (rightPtr != NULL)
      {
        left(row, col) += right(row, col);
      }
    }
  }

  __global__ void SumScore(double *score, int fullSize, int halfSize)
  {
    int index = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;

    for (int i = index; i < halfSize; i += stride)
    {
      score[i] += (i + halfSize < fullSize) ? score[i + halfSize] : 0;
    }
  }

  __global__ void ExtractValidPoints(
    float *sourceX, float *sourceY, float *sourceZ,
    float *extractedX, float *extractedY, float *extractedZ,
    int *validPoints, int validPointsNum)
  {
      int id = threadIdx.x + blockIdx.x * blockDim.x;
      int stride = blockDim.x * gridDim.x;

      for (int i = id; i < validPointsNum; i += stride)
      {
          extractedX[i] = sourceX[validPoints[i]];
          extractedY[i] = sourceY[validPoints[i]];
          extractedZ[i] = sourceZ[validPoints[i]];
      }
  }

  __global__ void TransformUseGpu(
    float *inputPointsX, float *inputPointsY, float *inputPointsZ,
    float *transformedPointsX, float *transformedPointsY, float *transformedPointsZ,
    int pointNum, MatrixDevice transform)
  {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    float x, y, z;

    for (int i = idx; i < pointNum; i += stride)
    {
      x = inputPointsX[i];
      y = inputPointsY[i];
      z = inputPointsZ[i];
      transformedPointsX[i] = transform(0, 0) * x + transform(0, 1) * y +
        transform(0, 2) * z + transform(0, 3);
      transformedPointsY[i] = transform(1, 0) * x + transform(1, 1) * y +
        transform(1, 2) * z + transform(1, 3);
      transformedPointsZ[i] = transform(2, 0) * x + transform(2, 1) * y +
        transform(2, 2) * z + transform(2, 3);
    }
  }

} // namespace gpu
