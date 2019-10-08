
cmake_minimum_required(VERSION 3.1)

project(enguerrand)

set(ENGUERRAND_VERSION_MAJOR 1)
set(ENGUERRAND_VERSION_MINOR 0)

option(WITH_CUDA "Whether CUDA is available." OFF)
option(BUILD_TEST "Whether to build unit tests." OFF)
option(WITH_DOC "Whether to enable generation of doxygen documentation." OFF)

enable_language(CXX)

if(${WITH_CUDA})
    enable_language(CUDA)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include("${CMAKE_SOURCE_DIR}/thirdparty/tbb/cmake/TBBBuild.cmake")
tbb_build(TBB_ROOT "${CMAKE_SOURCE_DIR}/thirdparty/tbb" CONFIG_DIR TBB_DIR)
find_package(TBB REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(Ceres REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS OpenGL Sql Core Widgets Gui Test REQUIRED)
find_package(Sophus REQUIRED)
find_package(libsvm REQUIRED)

pkg_check_modules(OSG REQUIRED IMPORTED_TARGET openscenegraph)
#pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

if(${WITH_DOC})
    find_package(Doxygen REQUIRED)
    add_subdirectory(doc)
endif()

add_subdirectory(app)
add_subdirectory(src)
if(${BUILD_TEST})
    add_subdirectory(tst)
endif()

