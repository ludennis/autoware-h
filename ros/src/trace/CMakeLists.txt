project(trace)
cmake_minimum_required(VERSION 3.0)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES trace
)

# Options

    option(TRACE_ENABLED "Enable tracing" ON)

# Targets

    add_library(trace
        source/utils.cpp
    )

# Flags

    target_compile_definitions(trace
        PUBLIC
            "TRACE_ENABLED=$<BOOL:${TRACE_ENABLED}>"
            "TRACE_STATIC_LEVEL=Trace::LEVEL_DEBUG"
    )

    target_compile_options(trace
        PRIVATE
            -fPIC
            -std=c++11
    )

# Includes

    target_include_directories(trace PUBLIC include)
