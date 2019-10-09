
add_library(
    enguerrand_core
    SHARED
    CalibrationData.h
    CalibrationData.cpp
    Histogram.h
    Histogram.cpp
    SE3Parameterization.h
    SE3Parameterization.cpp
    NullOdometry.cpp
    NullOdometry.h
    BAOdometry.cpp
    BAOdometry.h
    PFOdometry.cpp
    PFOdometry.h
    CircleFitter.cpp
    CircleFitter.h
    CirclesTracker.cpp
    CirclesTracker.h
    EdgeDetectionCPU.cpp
    EdgeDetectionCPU.h
    EKFOdometry.cpp
    EKFOdometry.h
    LineFitter.cpp
    LineFitter.h
    OdometryCode.cpp
    OdometryCode.h
    TrackedCircle.cpp
    TrackedCircle.h
)

target_link_libraries(
    enguerrand_core
    PUBLIC
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_calib3d
    Qt5::Core
    Threads::Threads
    Sophus::Sophus
)

target_link_libraries(
    enguerrand_core
    PRIVATE
    ceres
    libsvm
)

target_include_directories(enguerrand_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
