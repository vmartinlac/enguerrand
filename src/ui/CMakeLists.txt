
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
    ConfigDialog.h
    ConfigDialog.cpp
    ConfigDialog.ui
    CalibrationModel.h
    CalibrationModel.cpp
    CalibrationDialog.h
    CalibrationDialog.cpp
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

