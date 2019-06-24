
add_library(
    enguerrand_pipeline
    SHARED
    PipelineUtils.cpp
    PipelineUtils.h
    PipelineDescription.cpp
    PipelineDescription.h
    PipelineEngine.cpp
    PipelineEngine.h
    PipelineModule.cpp
    PipelineModule.h
    PipelineModuleWrapper.cpp
    PipelineModuleWrapper.h
    PipelinePort.cpp
    PipelinePort.h
    PipelineThread.cpp
    PipelineThread.h
    Semaphore.cpp
    Semaphore.h
)

target_link_libraries(
    enguerrand_pipeline
    PUBLIC
    opencv_core
    Sophus::Sophus
    Eigen3::Eigen
)

target_include_directories(enguerrand_pipeline INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

