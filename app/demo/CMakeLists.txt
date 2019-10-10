
add_executable(
    enguerrand_demo
    main.cpp
)

target_link_libraries(
    enguerrand_demo
    PUBLIC
    enguerrand_engine
    enguerrand_ui
)

install(TARGETS enguerrand_demo RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

