
add_library(
    enguerrand_engine
    SHARED
    EngineConfig.cpp
    EngineConfig.h
    Engine.cpp
    Engine.h
    EngineGraph.cpp
    EngineGraph.h
    Semaphore.cpp
    Semaphore.h
    EngineOutput.cpp
    EngineOutput.h
)

target_link_libraries(
    enguerrand_engine
    PUBLIC
    TBB::tbb
    TBB::tbbmalloc
    opencv_core
    opencv_imgproc
    Qt5::Core
    Qt5::Gui
    Threads::Threads
    enguerrand_video
    enguerrand_core
)

target_include_directories(enguerrand_engine INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

