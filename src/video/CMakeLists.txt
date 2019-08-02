
add_library(
    enguerrand_video
    SHARED
    GenICamInterface.cpp
    GenICamInterface.h
    GenICamVideoSource.cpp
    GenICamVideoSource.h
    FileVideoSource.cpp
    FileVideoSource.h
    RealsenseVideoSource.cpp
    RealsenseVideoSource.h
    VideoFrame.cpp
    VideoFrame.h
    VideoSource.cpp
    VideoSource.h
    ImageVideoSource.cpp
    ImageVideoSource.h
)

target_link_libraries(
    enguerrand_video
    PUBLIC
    opencv_core
    opencv_videoio
    realsense2::realsense2
)

target_include_directories(enguerrand_video INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

