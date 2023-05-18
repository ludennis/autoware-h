find_package(PCL)
find_package(CUDA)

if( PCL_FOUND AND CUDA_FOUND )

  set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

  if(NOT DEFINED CUDA_CAPABILITY_VERSION_CHECKER)
    set(CUDA_CAPABILITY_VERSION_CHECKER
    "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker")
  endif()
  execute_process(
    COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
    OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
    set(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
  else()
    set(CUDA_ARCH "sm_52")
  endif()
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++11;--ptxas-options=-v)
  set(SUBSYS_NAME ndt_gpu)
  set(SUBSYS_DESC "Point cloud ndt gpu library")

  include_directories(
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )

  cuda_add_library(NDT_GPU_LIB
    matrix_device.cu
    matrix_host.cu
    normal_distribution_transform.cu
    normal_distribution_transform_kernel.cu
    registration.cu
    voxel_grid.cu
    voxel_grid_kernel.cu
    symmetric_eigen_solver.cu
  )

  target_include_directories(NDT_GPU_LIB PRIVATE
    ${CUDA_INCLUDE_DIRS}
  )

  target_link_libraries(NDT_GPU_LIB
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
  )

else ()
  message(WARNING "ndt_gpu requires CUDA and PCL")
endif ()
