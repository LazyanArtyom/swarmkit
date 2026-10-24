# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

add_executable(swarmkit_tests
    test_arbiter.cpp
    test_basic.cpp
    test_backend_contract.cpp
    test_client_integration.cpp
    test_edge_attempt_journal.cpp
    test_command_preconditions.cpp
    test_config.cpp
    test_logger.cpp
    test_report_hub.cpp
    test_sim_backend.cpp
    test_mavlink_common.cpp
    test_swarm_client.cpp
    test_telemetry_manager.cpp
    test_telemetry_sink.cpp
    test_vecr.cpp
    test_vecr_v5_models.cpp
    test_vecr_truth_oracle.cpp
    test_vecr_agent_trial.cpp
    vecr_agent_trial.cpp
    vecr_truth_oracle.cpp
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
    swarmkit_agent
    swarmkit_evidence
    swarmkit_client
    Catch2::Catch2WithMain
)

swarmkit_set_warnings(swarmkit_tests)
swarmkit_enable_sanitizers(swarmkit_tests)

add_test(NAME swarmkit_tests COMMAND swarmkit_tests)

add_executable(swarmkit_vecr_agent_trial
    vecr_agent_trial_main.cpp
    vecr_agent_trial.cpp
    vecr_truth_oracle.cpp
)
target_link_libraries(swarmkit_vecr_agent_trial PRIVATE
    swarmkit_agent
    swarmkit_evidence
    swarmkit_client
)
swarmkit_set_warnings(swarmkit_vecr_agent_trial)
swarmkit_enable_sanitizers(swarmkit_vecr_agent_trial)

add_executable(swarmkit_vecr_v5_trial vecr_v5_trial.cpp)
target_link_libraries(swarmkit_vecr_v5_trial PRIVATE swarmkit_agent swarmkit_evidence swarmkit_client)
swarmkit_set_warnings(swarmkit_vecr_v5_trial)
swarmkit_enable_sanitizers(swarmkit_vecr_v5_trial)
