# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

add_executable(swarmkit-agent
    ${CMAKE_CURRENT_SOURCE_DIR}/agent/app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/agent/main.cpp
)
target_link_libraries(swarmkit-agent PRIVATE swarmkit_agent)
swarmkit_set_warnings(swarmkit-agent)
swarmkit_enable_sanitizers(swarmkit-agent)

add_executable(swarmkit-cli
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/main.cpp
)
target_link_libraries(swarmkit-cli PRIVATE swarmkit_client)
swarmkit_set_warnings(swarmkit-cli)
swarmkit_enable_sanitizers(swarmkit-cli)
