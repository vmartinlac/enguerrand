
add_executable(
    enguerrand
    main.cpp
)

target_link_libraries(
    enguerrand
    PUBLIC
    tbb
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    enguerrand_video
    enguerrand_slam
)

