add_library(
    enguerrand_slam
    SHARED
    SLAMPipeline.h
    SLAMPipeline.cpp
    VideoPort.h
    VideoPort.cpp
    EdgeCirclesPort.h
    EdgeCirclesPort.cpp
    VideoModule.h
    VideoModule.cpp
    EdgeDetectionCPU.h
    EdgeDetectionCPU.cpp
    EdgeDetectionGPU.h
    EdgeDetectionGPU.cu
    PoseLandmarksPort.h
    PoseLandmarksPort.cpp
    CirclesDetection.h
    CirclesDetection.cpp
    CircleFitter.h
    CircleFitter.cpp
    LineFitter.h
    LineFitter.cpp
    EKFSLAM.h
    EKFSLAM.cpp
)

target_link_libraries(
    enguerrand_slam
    PUBLIC
    enguerrand_video
    enguerrand_pipeline
    opencv_core
    Eigen3::Eigen
    PRIVATE
    opencv_cudaimgproc
    opencv_imgproc
    opencv_highgui # temporary
)

target_include_directories(enguerrand_slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

