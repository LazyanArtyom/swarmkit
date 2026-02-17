add_library(swarmkit_agent_runtime STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/agent_server.cpp
)

target_include_directories(swarmkit_agent_runtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../../generated/include>
)

target_link_libraries(swarmkit_agent_runtime PUBLIC
    swarmkit_core
    swarmkit_proto
)

swarmkit_set_warnings(swarmkit_agent_runtime)
swarmkit_enable_sanitizers(swarmkit_agent_runtime)