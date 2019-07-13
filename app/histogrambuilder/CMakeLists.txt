
add_executable(
    enguerrand_histogrambuilder
    main.cpp
)

target_link_libraries(
    enguerrand_histogrambuilder
    PUBLIC
    opencv_imgcodecs
    opencv_core
    Qt5::Core
    enguerrand_stuff
)

