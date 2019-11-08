
file(GLOB HEADER_FILES LIST_DIRECTORIES FALSE "*.h")
file(GLOB SOURCE_FILES LIST_DIRECTORIES FALSE "*.cpp")

add_library(
    enguerrand_video
    SHARED
    ${HEADER_FILES}
    ${SOURCE_FILES})

set_target_properties(enguerrand_video PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

target_link_libraries(
    enguerrand_video
    PUBLIC
    opencv_core
    opencv_videoio
    Qt5::Core
    realsense2::realsense2
    Sophus::Sophus
    Eigen3::Eigen)

target_include_directories(enguerrand_video INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/enguerrand/video>)

target_compile_definitions(enguerrand_video PUBLIC QT_NO_FOREACH)

install(
    TARGETS enguerrand_video
    EXPORT enguerrand-targets COMPONENT dev
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT bin
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT dev
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/enguerrand/video COMPONENT dev
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT bin)

