
add_library(
    enguerrand_stuff
    SHARED
    Histogram.h
    Histogram.cpp
)

target_link_libraries(
    enguerrand_stuff
    PUBLIC
    opencv_imgcodecs
    Qt5::Core
)

target_include_directories(enguerrand_stuff INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

