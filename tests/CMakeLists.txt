# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

add_executable(swarmkit_tests
    test_arbiter.cpp
    test_basic.cpp
    test_backend_contract.cpp
    test_clock_quality.cpp
    test_evidence_store.cpp
    test_state_quality_contract.cpp
    test_state_acceptance_engine.cpp
    test_state_acceptance_certificate.cpp
    test_state_acceptance_verifier.cpp
    test_client_integration.cpp
    test_command_preconditions.cpp
    test_config.cpp
    test_logger.cpp
    test_report_hub.cpp
    test_sim_backend.cpp
    test_mavlink_common.cpp
    test_phase3_experiment.cpp
    test_swarm_client.cpp
    test_telemetry_manager.cpp
    test_telemetry_sink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../apps/cli/telemetry_sink.cpp
)

target_include_directories(swarmkit_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/agent/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/../apps
    ${CMAKE_CURRENT_SOURCE_DIR}/../apps/cli
)

target_include_directories(swarmkit_tests SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/mavlink_c_library_v2
)

target_link_libraries(swarmkit_tests PRIVATE
    swarmkit_experiment
    Catch2::Catch2WithMain
)

swarmkit_set_warnings(swarmkit_tests)
swarmkit_enable_sanitizers(swarmkit_tests)

add_test(NAME swarmkit_tests COMMAND swarmkit_tests)
