
add_library(
    enguerrand_video
    SHARED
    VideoInput.h
    VideoInput.cpp
)

target_link_libraries(
    enguerrand_video
    PUBLIC
    opencv_core
    Sophus::Sophus
    Eigen3::Eigen
    PRIVATE
    realsense2::realsense2
)

target_include_directories(enguerrand_video INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

