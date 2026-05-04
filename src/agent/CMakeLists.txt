# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

add_library(swarmkit_agent STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/server.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arbiter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/active_goal_supervisor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/report_hub.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/telemetry_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/backends/sim_backend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/backends/mavlink_backend.cpp
)

target_include_directories(swarmkit_agent PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(swarmkit_agent PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../core
)

target_include_directories(swarmkit_agent SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/mavlink_c_library_v2
)

target_link_libraries(swarmkit_agent PUBLIC
    swarmkit_core
    swarmkit_proto
    yaml-cpp::yaml-cpp
)

set_target_properties(swarmkit_agent PROPERTIES EXPORT_NAME agent)

swarmkit_set_warnings(swarmkit_agent)
swarmkit_enable_sanitizers(swarmkit_agent)
