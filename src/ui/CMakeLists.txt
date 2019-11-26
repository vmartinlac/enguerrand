
set(
    SOURCE_FILES
    AboutDialog.cpp
    AboutDialog.ui
    ConfigDialog2.ui
    ConfigDialog.cpp
    ConfigDialog.ui
    MainWindow.cpp
    VideoWidget.cpp
    ViewerWidgetBase.cpp
    ViewerWidget.cpp
    resources/resources_ui.qrc)

set(
    HEADER_FILES
    AboutDialog.h
    ConfigDialog.h
    MainWindow.h
    VideoWidget.h
    ViewerWidgetBase.h
    ViewerWidget.h)

add_library(
    enguerrand_ui
    SHARED
    ${HEADER_FILES}
    ${SOURCE_FILES})

set_target_properties(enguerrand_ui PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

target_link_libraries(
    enguerrand_ui
    PUBLIC
    Qt5::Sql
    Qt5::Widgets
    enguerrand_engine
    PkgConfig::OSG
    opencv_core)

configure_file(BuildInfo.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/BuildInfo.h" @ONLY)

target_include_directories(
    enguerrand_ui
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/enguerrand/ui/>
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

install(
    TARGETS enguerrand_ui
    EXPORT enguerrand-targets COMPONENT dev
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/enguerrand/ui COMPONENT dev
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT bin
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT dev
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT bin)

