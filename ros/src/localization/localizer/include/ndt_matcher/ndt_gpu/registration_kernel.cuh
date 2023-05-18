#ifndef REGISTRATION_KERNEL_H_
#define REGISTRATION_KERNEL_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>

namespace gpu
{

  template <typename T>
  __global__ void ConvertInputPoints(T *input,
    float *outputPointsX, float *outputPointsY, float *outputPointsZ,
    int pointsNum)
  {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;

    for (int i = idx; i < pointsNum; i += stride)
    {
      T tmp = input[i];
      outputPointsX[i] = tmp.x;
      outputPointsY[i] = tmp.y;
      outputPointsZ[i] = tmp.z;
    }
  }

} // namespace gpu

#endif // REGISTRATION_KERNEL_H_
