
add_executable(
    enguerrand_test_fitting
    fitting.cpp
)

target_link_libraries(
    enguerrand_test_fitting
    PUBLIC
    Qt5::Test
    enguerrand_engine
)

add_executable(
    enguerrand_test_odometry
    odometry.cpp
)

target_link_libraries(
    enguerrand_test_odometry
    PUBLIC
    Qt5::Test
    enguerrand_engine
)
