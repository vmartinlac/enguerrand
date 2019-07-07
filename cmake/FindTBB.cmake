include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(TBB DEFAULT_MSG TBB_LIBRARY TBB_INCLUDE_DIR)

add_library(tbb INTERFACE)
target_link_libraries(tbb INTERFACE ${TBB_LIBRARY})
target_include_directories(tbb INTERFACE ${TBB_INCLUDE_DIR})

