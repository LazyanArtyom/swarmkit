# tests/CMakeLists.txt

add_executable(swarmkit_tests
    test_basic.cpp
)

target_link_libraries(swarmkit_tests PRIVATE
    swarmkit_core
    Catch2::Catch2WithMain
)

target_include_directories(swarmkit_tests PRIVATE
    ${CMAKE_BINARY_DIR}/generated/include
)

swarmkit_set_warnings(swarmkit_tests)
swarmkit_enable_sanitizers(swarmkit_tests)

add_test(NAME swarmkit_tests COMMAND swarmkit_tests)