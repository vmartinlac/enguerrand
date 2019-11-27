
add_executable(
    enguerrand_test_staticodometry
    Test.h
    Test.cpp
)

target_link_libraries(
    enguerrand_test_staticodometry
    PUBLIC
    Qt5::Test
    enguerrand_engine
    opencv_imgproc
)

