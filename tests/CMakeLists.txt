add_executable(swarmkit_tests
    test_arbiter.cpp
    test_basic.cpp
    test_sim_backend.cpp
)

target_link_libraries(swarmkit_tests PRIVATE
    swarmkit_agent
    Catch2::Catch2WithMain
)

swarmkit_set_warnings(swarmkit_tests)
swarmkit_enable_sanitizers(swarmkit_tests)

add_test(NAME swarmkit_tests COMMAND swarmkit_tests)
