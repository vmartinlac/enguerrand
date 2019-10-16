
add_library(
    enguerrand_tools
    STATIC
    DataDirIterator.cpp
    DataDirIterator.h
)

target_link_libraries(
    enguerrand_tools
    PUBLIC
    opencv_core
    opencv_imgcodecs
    Qt5::Core
)

target_include_directories(enguerrand_tools INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(
    TARGETS enguerrand_tools
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

