
add_executable(
    test_fitting
    main.cpp
)

target_link_libraries(
    test_fitting
    PUBLIC
    Qt5::Test
    enguerrand_slam
)
