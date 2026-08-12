# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

add_library(swarmkit_evidence STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/execution_log.cpp
)

target_include_directories(swarmkit_evidence PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(swarmkit_evidence PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../core
)

target_link_libraries(swarmkit_evidence PUBLIC
  swarmkit_core
  swarmkit_proto
)

set_target_properties(swarmkit_evidence PROPERTIES EXPORT_NAME evidence)

swarmkit_set_warnings(swarmkit_evidence)
swarmkit_enable_sanitizers(swarmkit_evidence)
