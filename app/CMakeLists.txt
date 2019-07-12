
add_executable(
    enguerrand
    main.cpp
    Semaphore.h
    Semaphore.cpp
    Engine.h
    Engine.cpp
    EngineGraph.h
    EngineGraph.cpp
    EdgeDetectionCPU.h
    EdgeDetectionCPU.cpp
    CirclesTracker.h
    CirclesTracker.cpp
    TrackedCircle.h
    TrackedCircle.cpp
    OdometryCode.h
    OdometryCode.cpp
    EKFOdometry.h
    EKFOdometry.cpp
)

target_link_libraries(
    enguerrand
    PUBLIC
    TBB::tbb
    TBB::tbbmalloc
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    Threads::Threads
    Sophus::Sophus
    enguerrand_video
)

