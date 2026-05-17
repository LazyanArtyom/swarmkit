# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

add_executable(swarmkit_tests
    test_arbiter.cpp
    test_basic.cpp
    test_client_integration.cpp
    test_config.cpp
    test_logger.cpp
    test_report_hub.cpp
    test_sim_backend.cpp
    test_mavlink_common.cpp
    test_swarm_client.cpp
    test_trajectory_io.cpp
)

target_include_directories(swarmkit_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/agent/backends
)

target_include_directories(swarmkit_tests SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/mavlink_c_library_v2
)

target_link_libraries(swarmkit_tests PRIVATE
    swarmkit_agent
    swarmkit_client
    Catch2::Catch2WithMain
)

swarmkit_set_warnings(swarmkit_tests)
swarmkit_enable_sanitizers(swarmkit_tests)

add_test(NAME swarmkit_tests COMMAND swarmkit_tests)
