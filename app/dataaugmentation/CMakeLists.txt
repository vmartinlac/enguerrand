
add_executable(
    enguerrand_dataaugmentation
    main.cpp
    VariantGenerator.h
    VariantGenerator.cpp
)

target_link_libraries(
    enguerrand_dataaugmentation
    PUBLIC
    opencv_imgcodecs
    Qt5::Core
    enguerrand_core
)

install(TARGETS enguerrand_dataaugmentation RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

