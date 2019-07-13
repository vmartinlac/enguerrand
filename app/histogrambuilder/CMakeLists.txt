
add_executable(
    enguerrand_histogrambuilder
    main.cpp
)

target_link_libraries(enguerrand_histogrambuilder PUBLIC opencv_imgcodecs Qt5::Core enguerrand_stuff)

