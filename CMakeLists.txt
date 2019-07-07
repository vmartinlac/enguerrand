cmake_minimum_required(VERSION 3.1)

project(enguerrand)

set(ENGUERRAND_VERSION_MAJOR 1)
set(ENGUERRAND_VERSION_MINOR 0)
set(ENGUERRAND_VERSION_REVISION 0)

enable_language(CXX)
#enable_language(CUDA)

include(GNUInstallDirs)
include(GenerateExportHeader)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(PkgConfig REQUIRED)
find_package(FFmpeg REQUIRED)
#find_package(gRPC REQUIRED)
find_package(Ceres REQUIRED)
find_package(realsense2 REQUIRED)
find_package(libusb REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(OSG REQUIRED)
find_package(Qt5 COMPONENTS OpenGL Sql Core Widgets Gui Network Test REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)
#find_package(EMBB REQUIRED)

add_subdirectory(app)
add_subdirectory(src)
#add_subdirectory(tst)

