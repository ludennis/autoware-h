#ifndef __NDT_GPU_MEMORY_H__
#define __NDT_GPU_MEMORY_H__

#include <cassert>
#include <memory>

namespace gpu
{
  template <typename T>
  static inline std::shared_ptr<T> AllocateCudaMemory(const int count)
  {
    T * pointer = nullptr;
    checkCudaErrors(cudaMalloc(&pointer, sizeof(T) * count));
    if (pointer != nullptr)
          return std::shared_ptr<T>(pointer, [] (T * pointer) {
          assert(pointer != nullptr);
          checkCudaErrors(cudaFree(pointer));
        });
    else
      return nullptr;
  }

  static inline size_t GetCudaMemoryFree()
  {
    size_t freeMem = 0;
    size_t totalMem = 0;
    cudaMemGetInfo(&freeMem, &totalMem);
    printf("freeMem: %lu\n", freeMem);
    return freeMem;
  }
} // namespace gpu

#ifdef PROFILE_CUDA_MEMORY
  #define PROFILE_CUDA_MEMORY_BEGIN(tag) \
    const int _cudaFreeMemoryBefore ## tag = gpu::GetCudaMemoryFree()

  #define PROFILE_CUDA_MEMORY_END(tag) \
    const int _cudaFreeMemoryAfter ## tag = gpu::GetCudaMemoryFree(); \
    printf(#tag " allocates %d bytes\n", \
      _cudaFreeMemoryBefore ## tag - _cudaFreeMemoryAfter ## tag)
#else
  #define PROFILE_CUDA_MEMORY_BEGIN(tag)
  #define PROFILE_CUDA_MEMORY_END(tag)
#endif // PROFILE_CUDA_MEMORY

#endif // __NDT_GPU_MEMORY_H__
