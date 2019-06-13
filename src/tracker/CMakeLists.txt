add_library(
    enguerrand_tracker
    SHARED
    Tracker.h
    Tracker.cpp
    TrackerGPUImpl.h
    TrackerGPUImpl.cu
    TrackerCPUImpl.h
    TrackerCPUImpl.cpp
    CircleFitter.h
    CircleFitter.cpp
    LineFitter.h
    LineFitter.cpp
)

target_link_libraries(
    enguerrand_tracker
    PUBLIC
    opencv_core
    Eigen3::Eigen
    PRIVATE
    opencv_cudaimgproc
    opencv_imgproc
    opencv_highgui # temporary
)

target_include_directories(enguerrand_tracker INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

