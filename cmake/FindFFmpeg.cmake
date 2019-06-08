
pkg_search_module(LIBAVFORMAT REQUIRED libavformat)
pkg_search_module(LIBAVCODEC REQUIRED libavcodec)
pkg_search_module(LIBAVDEVICE REQUIRED libavdevice)
pkg_search_module(LIBAVFILTER REQUIRED libavfilter)
pkg_search_module(LIBAVUTIL REQUIRED libavutil)
pkg_search_module(LIBSWRESAMPLE REQUIRED libswresample)
pkg_search_module(LIBSWSCALE REQUIRED libswscale)


add_library(FFmpeg INTERFACE)

target_include_directories(
    FFmpeg INTERFACE
    ${LIBAVFORMAT_INCLUDE_DIRS}
    ${LIBAVCODEC_INCLUDE_DIRS}
    ${LIBAVDEVICE_INCLUDE_DIRS}
    ${LIBAVFILTER_INCLUDE_DIRS}
    ${LIBAVUTIL_INCLUDE_DIRS}
    ${LIBSWRESAMPLE_INCLUDE_DIRS}
    ${LIBSWSCALE_INCLUDE_DIRS}
)

target_link_libraries(
    FFmpeg INTERFACE
    ${LIBAVFORMAT_LDFLAGS}
    ${LIBAVCODEC_LDFLAGS}
    ${LIBAVDEVICE_LDFLAGS}
    ${LIBAVFILTER_LDFLAGS}
    ${LIBAVUTIL_LDFLAGS}
    ${LIBSWRESAMPLE_LDFLAGS}
    ${LIBSWSCALE_LDFLAGS}
)
