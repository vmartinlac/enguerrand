
add_library(
    enguerrand_engine
    SHARED
    BAOdometry.cpp
    BAOdometry.h
    CalibrationData.cpp
    CalibrationData.h
    CircleFitter.cpp
    CircleFitter.h
    CirclesTracker.cpp
    CirclesTracker.h
    EdgeDetectionCPU.cpp
    EdgeDetectionCPU.h
    EKFOdometry.cpp
    EKFOdometry.h
    EngineConfig.cpp
    EngineConfig.h
    Engine.cpp
    EngineGraph.cpp
    EngineGraph.h
    Engine.h
    LineFitter.cpp
    LineFitter.h
    OdometryCode.cpp
    OdometryCode.h
    Semaphore.cpp
    Semaphore.h
    TrackedCircle.cpp
    TrackedCircle.h
    EngineOutput.cpp
    EngineOutput.h
)

target_link_libraries(
    enguerrand_engine
    PUBLIC
    ceres
    TBB::tbb
    TBB::tbbmalloc
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
    opencv_calib3d
    Qt5::Core
    Threads::Threads
    Eigen3::Eigen
    Sophus::Sophus
    enguerrand_video
    enguerrand_stuff
)

target_include_directories(enguerrand_engine INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

