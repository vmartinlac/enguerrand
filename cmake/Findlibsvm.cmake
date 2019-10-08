include(FindPackageHandleStandardArgs)

find_library(LIBSVM_LIBRARY svm)
find_path(LIBSVM_INCLUDE_DIR svm.h PATH_SUFFIXES libsvm)

set(LIBSVM_LIBRARIES ${LIBSVM_LIBRARY})
set(LIBSVM_INCLUDE_DIRS ${LIBSVM_INCLUDE_DIR})

find_package_handle_standard_args(
    libsvm DEFAULT_MSG LIBSVM_INCLUDE_DIRS LIBSVM_LIBRARIES)

if(${libsvm_FOUND})
    add_library(libsvm INTERFACE)
    target_link_libraries(libsvm INTERFACE ${LIBSVM_LIBRARIES})
    target_include_directories(libsvm INTERFACE ${LIBSVM_INCLUDE_DIRS})
endif()

