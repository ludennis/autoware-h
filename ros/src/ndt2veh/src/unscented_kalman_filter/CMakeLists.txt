cmake_minimum_required(VERSION 2.8.3)
project(unscented_kalman_filter)

add_compile_options(-std=c++11)

add_library(${PROJECT_NAME}
    src/diag.cpp
    src/mod.cpp
    src/mrdivide.cpp
    src/rtGetInf.cpp
    src/rtGetNaN.cpp
    src/rt_nonfinite.cpp
    src/sum.cpp
    src/UnscentedKalmanFilter.cpp
    src/UnscentedKalmanFilter_initialize.cpp
    src/UnscentedKalmanFilter_terminate.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
)
