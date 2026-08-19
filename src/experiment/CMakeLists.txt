# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

add_library(swarmkit_experiment STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/manual_runtime.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/scripted_backend.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/fault_injection.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/telemetry_replay.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/state_acceptance_experiment.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/bootstrap_statistics.cpp
)

target_include_directories(swarmkit_experiment PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(swarmkit_experiment PUBLIC
  swarmkit_agent
  swarmkit_evidence
  swarmkit_client
)

set_target_properties(swarmkit_experiment PROPERTIES EXPORT_NAME experiment)

swarmkit_set_warnings(swarmkit_experiment)
swarmkit_enable_sanitizers(swarmkit_experiment)
