# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

add_library(swarmkit_core STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/logger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/evidence_store.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/state_quality_contract.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/state_acceptance_engine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/state_acceptance_certificate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/state_acceptance_verifier.cpp
)

target_include_directories(swarmkit_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(swarmkit_core PUBLIC
    fmt::fmt
    spdlog::spdlog
)

set_target_properties(swarmkit_core PROPERTIES EXPORT_NAME core)

swarmkit_set_warnings(swarmkit_core)
swarmkit_enable_sanitizers(swarmkit_core)
