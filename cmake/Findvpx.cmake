
pkg_search_module(LIBVPX REQUIRED vpx)

add_library(vpx INTERFACE)
target_include_directories(vpx INTERFACE ${LIBVPX_INCLUDE_DIRS})
target_link_libraries(vpx INTERFACE ${LIBVPX_LDFLAGS})

