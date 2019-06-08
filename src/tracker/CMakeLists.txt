add_library(
    enguerrand_tracker
    SHARED
    Tracker.h
    Tracker.cpp
    TrackerImpl.h
    TrackerImpl.cu
)

target_link_libraries(
    enguerrand_tracker
    PUBLIC
    opencv_core
    PRIVATE
    opencv_cudaimgproc
    opencv_imgproc
    opencv_highgui # temporary
)

target_include_directories(enguerrand_tracker INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

