
add_library(
    enguerrand_ui
    SHARED
    resources/resources_ui.qrc
    AboutDialog.ui
    AboutDialog.cpp
    AboutDialog.h
    MainWindow.cpp
    MainWindow.h
    VideoWidget.cpp
    VideoWidget.h
    ViewerWidgetBase.cpp
    ViewerWidgetBase.h
    ViewerWidget.cpp
    ViewerWidget.h
    ConfigDialog.h
    ConfigDialog.cpp
    ConfigDialog.ui
)

target_link_libraries(
    enguerrand_ui
    PUBLIC
    Qt5::Sql
    Qt5::Widgets
    enguerrand_engine
    PkgConfig::OSG
    opencv_core
)

target_include_directories(enguerrand_ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

configure_file(BuildInfo.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/BuildInfo.h" @ONLY)
target_include_directories(enguerrand_ui PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/include")

install(TARGETS enguerrand_ui RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

