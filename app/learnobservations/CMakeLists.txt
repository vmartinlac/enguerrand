
add_executable(
    enguerrand_learnobservations
    main.cpp
    DataDirIterator.h
    DataDirIterator.cpp
)

target_link_libraries(
    enguerrand_learnobservations
    PUBLIC
    opencv_imgcodecs
    Qt5::Core
    enguerrand_core
)

install(TARGETS enguerrand_learnobservations RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

