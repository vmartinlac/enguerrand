
pkg_search_module(OSG REQUIRED openscenegraph)

add_library(openscenegraph INTERFACE)
target_include_directories(openscenegraph INTERFACE ${OSG_INCLUDE_DIRS})
target_link_libraries(openscenegraph INTERFACE ${OSG_LDFLAGS})

