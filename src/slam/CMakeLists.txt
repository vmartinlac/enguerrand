
add_library(
    enguerrand_slam
    SHARED
    Semaphore.cpp
    Semaphore.h
    SLAMEngine.cpp
    SLAMEngine.h
    SLAMModule.cpp
    SLAMModule.h
    SLAMModuleIds.h
    SLAMModuleWrapper.cpp
    SLAMModuleWrapper.h
    SLAMPipeline.cpp
    SLAMPipeline.h
    SLAMPort.h
    SLAMPortTypes.h
    SLAMThread.cpp
    SLAMThread.h
)

target_link_libraries(
    enguerrand_slam
    PUBLIC
    opencv_core
    Sophus::Sophus
    Eigen3::Eigen
)

target_include_directories(enguerrand_slam INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

