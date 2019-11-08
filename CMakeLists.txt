
cmake_minimum_required(VERSION 3.1)

project(enguerrand)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(ENGUERRAND_VERSION_MAJOR 1)
set(ENGUERRAND_VERSION_MINOR 0)
set(ENGUERRAND_VERSION_REVISION 0)

set(ENGUERRAND_VERSION "${ENGUERRAND_VERSION_MAJOR}.${ENGUERRAND_VERSION_MINOR}.${ENGUERRAND_VERSION_REVISION}")

option(WITH_CUDA "Whether CUDA is available." OFF)
option(WITH_DOXYGEN "Whether to enable generation of doxygen documentation." OFF)
option(BUILD_TEST "Whether to build unit tests." OFF)

if(${WITH_DOXYGEN})
set(DOCUMENTATION_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/doxygen_documentation/" CACHE PATH "Path to directory in which to generate documentation.")
endif()

enable_language(CXX)

if(${WITH_CUDA})
    enable_language(CUDA)
    set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS WITH_CUDA)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_SOURCE_DIR}/cmake")

include("${CMAKE_SOURCE_DIR}/thirdparty/tbb/cmake/TBBBuild.cmake")
tbb_build(TBB_ROOT "${CMAKE_SOURCE_DIR}/thirdparty/tbb" CONFIG_DIR TBB_DIR)
find_package(TBB REQUIRED)

find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(Ceres REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS OpenGL Sql Core Widgets Gui Test REQUIRED)
find_package(Sophus REQUIRED)

pkg_check_modules(OSG REQUIRED IMPORTED_TARGET openscenegraph)
#pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

if(${WITH_DOXYGEN})
    find_package(Doxygen REQUIRED)
    add_subdirectory(doc)
endif()

add_subdirectory(app)
add_subdirectory(src)
if(${BUILD_TEST})
    add_subdirectory(tst)
endif()

configure_package_config_file("cmake/enguerrand-config.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/enguerrand-config.cmake" INSTALL_DESTINATION "${CMAKE_INSTALL_DATADIR}/enguerrand/cmake")

install(EXPORT enguerrand-targets DESTINATION "${CMAKE_INSTALL_DATADIR}/enguerrand/cmake")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/enguerrand-config.cmake" DESTINATION "${CMAKE_INSTALL_DATADIR}/enguerrand/cmake")

set(CPACK_PACKAGE_VERSION_MAJOR ${ENGUERRAND_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${ENGUERRAND_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${ENGUERRAND_VERSION_REVISION})
include(CPack)

