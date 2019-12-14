
file(GLOB HEADER_FILES LIST_DIRECTORIES FALSE "*.h")
file(GLOB SOURCE_FILES LIST_DIRECTORIES FALSE "*.cpp")

add_library(
    enguerrand_engine
    SHARED
    ${HEADER_FILES}
    ${SOURCE_FILES})

set_target_properties(enguerrand_engine PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

target_link_libraries(
    enguerrand_engine
    PUBLIC
    TBB::tbb
    TBB::tbbmalloc
    opencv_core
    opencv_imgproc
    Qt5::Core
    #Qt5::Gui
    Threads::Threads
    enguerrand_video
    enguerrand_core)

target_include_directories(
    enguerrand_engine INTERFACE
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/enguerrand/engine>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

install(
    TARGETS enguerrand_engine
    EXPORT enguerrand-targets COMPONENT dev
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT bin
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT dev
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/enguerrand/engine COMPONENT dev
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT bin)

