
add_library(
    enguerrand_ekfslam
    SHARED
    EKFSLAMEngine.h
    EKFSLAMEngine.cpp
)

target_link_libraries(
    enguerrand_ekfslam
    PUBLIC
    enguerrand_slam
    opencv_core
    ceres
)

target_include_directories(enguerrand_ekfslam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

