cmake_minimum_required(VERSION 3.1)

project(enguerrand)

option(WITH_CUDA "Whether CUDA is available." OFF)

enable_language(CXX)

if(${WITH_CUDA})
    enable_language(CUDA)
endif()

include(GNUInstallDirs)
#include(GenerateExportHeader)

set(ENGUERRAND_VERSION_MAJOR 1)
set(ENGUERRAND_VERSION_MINOR 0)
set(ENGUERRAND_VERSION_REVISION 0)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(Ceres REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS OpenGL Sql Core Widgets Gui Network Test REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)

pkg_check_modules(OSG REQUIRED IMPORTED_TARGET openscenegraph)
#pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

add_subdirectory(app)
add_subdirectory(src)
add_subdirectory(tst)

