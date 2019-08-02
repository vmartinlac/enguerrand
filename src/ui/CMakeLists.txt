
add_library(
    enguerrand_ui
    SHARED
    MainWindow.cpp
    MainWindow.h
    VideoWidget.cpp
    VideoWidget.h
    ViewerWidgetBase.cpp
    ViewerWidgetBase.h
    ViewerWidget.cpp
    ViewerWidget.h
)

target_link_libraries(
    enguerrand_ui
    PUBLIC
    Qt5::Widgets
    enguerrand_engine
    PkgConfig::OSG
)

target_include_directories(enguerrand_ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

