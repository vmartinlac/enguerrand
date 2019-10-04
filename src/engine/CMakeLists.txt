
add_library(
    enguerrand_engine
    SHARED
    SE3Parameterization.h
    SE3Parameterization.cpp
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
    "${CMAKE_SOURCE_DIR}/thirdparty/libsvm/svm.h"
    "${CMAKE_SOURCE_DIR}/thirdparty/libsvm/svm.cpp"
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
    Qt5::Gui
    Threads::Threads
    enguerrand_video
    enguerrand_stuff
)

target_include_directories(enguerrand_engine PRIVATE "${CMAKE_SOURCE_DIR}/thirdparty/libsvm/")

target_include_directories(enguerrand_engine INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

