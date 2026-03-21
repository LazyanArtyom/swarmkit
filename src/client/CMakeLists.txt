# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

add_library(swarmkit_client STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/client.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/swarm_client.cpp
)

target_include_directories(swarmkit_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(swarmkit_client PUBLIC
  swarmkit_core
  swarmkit_proto
)

set_target_properties(swarmkit_client PROPERTIES EXPORT_NAME client)

swarmkit_set_warnings(swarmkit_client)
swarmkit_enable_sanitizers(swarmkit_client)
