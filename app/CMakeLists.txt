
add_executable(
    enguerrand
    main.cpp
    Semaphore.h
    Semaphore.cpp
    EngineFrame.h
    EngineFrame.cpp
    Engine.h
    Engine.cpp
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
    enguerrand_video
    enguerrand_slam
)

