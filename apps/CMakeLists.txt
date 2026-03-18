add_executable(swarmkit-agent
    ${CMAKE_CURRENT_SOURCE_DIR}/agent/main.cpp
)
target_link_libraries(swarmkit-agent PRIVATE swarmkit_agent)
swarmkit_set_warnings(swarmkit-agent)
swarmkit_enable_sanitizers(swarmkit-agent)

add_executable(swarmkit-cli
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/main.cpp
)
target_link_libraries(swarmkit-cli PRIVATE swarmkit_client)
swarmkit_set_warnings(swarmkit-cli)
swarmkit_enable_sanitizers(swarmkit-cli)
