
add_executable(
    enguerrand_learnhistograms
    main.cpp
    ObservationValidator.h
    ObservationValidator.cpp
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
    opencv_ml
    Qt5::Core
    enguerrand_core
)

