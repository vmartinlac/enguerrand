
add_executable(
    enguerrand_test_odometry
    TestOdometry.h
    TestOdometry.cpp
)

target_link_libraries(
    enguerrand_test_odometry
    PUBLIC
    Qt5::Test
    enguerrand_engine
)

