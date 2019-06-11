
pkg_search_module(LIBUSB REQUIRED libusb-1.0)

add_library(libusb INTERFACE)
target_include_directories(libusb INTERFACE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(libusb INTERFACE ${LIBUSB_LDFLAGS})

