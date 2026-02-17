add_executable(swarmkit_agent
    ${CMAKE_CURRENT_SOURCE_DIR}/agent/main.cpp
)
target_link_libraries(swarmkit_agent PRIVATE swarmkit_agent_runtime)
swarmkit_set_warnings(swarmkit_agent)
swarmkit_enable_sanitizers(swarmkit_agent)

add_executable(swarmkit_cli
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/main.cpp
)
target_link_libraries(swarmkit_cli PRIVATE swarmkit_core swarmkit_proto)
swarmkit_set_warnings(swarmkit_cli)
swarmkit_enable_sanitizers(swarmkit_cli)