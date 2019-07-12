
add_library(
    enguerrand_slam
    SHARED
    #EdgeDetectionCPU.h
    #EdgeDetectionCPU.cpp
    #CirclesDetection.h
    #CirclesDetection.cpp
    CircleFitter.h
    CircleFitter.cpp
    LineFitter.h
    LineFitter.cpp
)

target_link_libraries(
    enguerrand_slam
    PUBLIC
    enguerrand_video
    Eigen3::Eigen
    opencv_core
    opencv_imgproc
    opencv_highgui
)

target_include_directories(enguerrand_slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

