
add_library(
    enguerrand_slam
    SHARED
    SLAMEngine.h
    SLAMEngine.cpp
    SLAMFrame.h
    SLAMFrame.cpp
    Semaphore.h
    Semaphore.cpp
    SLAMModule.h
    SLAMModule.cpp
    SLAMThread.h
    SLAMThread.cpp
)

target_link_libraries(
    enguerrand_slam
    PUBLIC
    opencv_core
    Sophus::Sophus
    Eigen3::Eigen
)

target_include_directories(enguerrand_slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

