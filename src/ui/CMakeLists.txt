
add_library(
    enguerrand_ui
    SHARED
    MainWindow.cpp
    MainWindow.h
)

target_link_libraries(
    enguerrand_ui
    PUBLIC
    Qt5::Widgets
    enguerrand_engine
    openscenegraph
)

target_include_directories(enguerrand_ui INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

