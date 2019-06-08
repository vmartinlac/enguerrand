
add_library(
    enguerrand_fastslam
    SHARED
    FastSLAMEngine.h
    FastSLAMEngine.cpp
)

target_link_libraries(
    enguerrand_fastslam
    PUBLIC
    enguerrand_slam
    opencv_core
)

target_include_directories(enguerrand_fastslam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

