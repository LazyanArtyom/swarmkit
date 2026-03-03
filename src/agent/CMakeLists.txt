add_library(swarmkit_agent_runtime STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/agent_server.cpp
)

target_include_directories(swarmkit_agent_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(swarmkit_agent_runtime PUBLIC
  swarmkit_core
  swarmkit_proto
)

set_target_properties(swarmkit_agent_runtime PROPERTIES EXPORT_NAME agent_runtime)

swarmkit_set_warnings(swarmkit_agent_runtime)
swarmkit_enable_sanitizers(swarmkit_agent_runtime)