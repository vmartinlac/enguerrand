
file(GLOB HEADER_FILES LIST_DIRECTORIES FALSE "*.h")
file(GLOB SOURCE_FILES LIST_DIRECTORIES FALSE "*.cpp")

add_library(
    enguerrand_core
    SHARED
    ${HEADER_FILES}
    ${SOURCE_FILES})

set_target_properties(enguerrand_core PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

target_link_libraries(
    enguerrand_core
    PUBLIC
    opencv_core
    opencv_imgproc
    opencv_ml
    opencv_imgcodecs
    opencv_calib3d
    Qt5::Core
    Threads::Threads
    Sophus::Sophus
    PRIVATE
    ceres)

target_include_directories(
    enguerrand_core
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/enguerrand/core>)

install(
    TARGETS enguerrand_core
    EXPORT enguerrand-targets COMPONENT dev
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/enguerrand/core COMPONENT dev
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT bin
    LIBRARY DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT bin
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT dev)

if(${WITH_CUDA})

    add_library(
        enguerrand_core_cu
        SHARED
        EdgeDetectionGPU.cu
        EdgeDetectionGPU.h)

    target_link_libraries(enguerrand_core_cu PUBLIC opencv_core opencv_cudaimgproc opencv_cudafilters Sophus::Sophus)

    target_link_libraries(enguerrand_core PRIVATE enguerrand_core_cu)

endif()

