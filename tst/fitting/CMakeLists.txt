
add_executable(
    enguerrand_test_fitting
    TestFitting.h
    TestFitting.cpp
)

target_link_libraries(
    enguerrand_test_fitting
    PUBLIC
    Qt5::Test
    enguerrand_engine
)

