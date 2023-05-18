find_package(PCL REQUIRED)
find_package(CUDA)

add_subdirectory(ndt_gpu)

if(CUDA_FOUND)
  add_definitions(-DCUDA_FOUND)
endif()

add_library(NDT_MATCHER_LIB
  ${CMAKE_CURRENT_SOURCE_DIR}/ndt_matcher_cpu.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ndt_matcher_gpu.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ndt_matcher.cpp
  ${PROJECT_SOURCE_DIR}/include/common/pose.h
)

target_compile_options(NDT_MATCHER_LIB
  PRIVATE
  -std=c++11
  -Wall
  -Werror
)

target_link_libraries(NDT_MATCHER_LIB
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  NDT_GPU_LIB
)

if(CUDA_FOUND)
  target_include_directories(NDT_MATCHER_LIB PRIVATE
    ${CUDA_INCLUDE_DIRS}
  )
endif()
