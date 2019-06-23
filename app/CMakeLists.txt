
add_executable(
    enguerrand
    main.cpp
)

target_link_libraries(
    enguerrand
    PUBLIC
    opencv_core
    enguerrand_slam
)


## TEMPORARY
target_link_libraries(enguerrand PUBLIC opencv_imgcodecs opencv_imgproc)

