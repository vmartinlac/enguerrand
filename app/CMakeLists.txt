
add_executable(
    enguerrand
    main.cpp
)

target_link_libraries(
    enguerrand
    PUBLIC
    TBB::tbb
    TBB::tbbmalloc
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    enguerrand_video
    enguerrand_slam
)

