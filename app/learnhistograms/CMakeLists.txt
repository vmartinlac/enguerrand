
add_executable(
    enguerrand_learnhistograms
    main.cpp
    Histogram.h
    Histogram.cpp
    DataDirIterator.h
    DataDirIterator.cpp
)

target_link_libraries(
    enguerrand_learnhistograms
    PUBLIC
    opencv_imgcodecs
    opencv_core
    Qt5::Core
    libsvm
)

