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
    test_cli_edge_options.cpp
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
    test_vecr_sensor_models.cpp
    test_vecr_truth_oracle.cpp
    test_vecr_agent_trial.cpp
    vecr_agent_trial.cpp
    vecr_truth_oracle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../apps/cli/telemetry_sink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../apps/cli/options.cpp
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

# One production method, plus a non-installed full-restart research executable.
add_executable(swarmkit_vecr_trial vecr_digital_trial.cpp)
target_link_libraries(swarmkit_vecr_trial PRIVATE swarmkit_agent swarmkit_evidence swarmkit_client)
swarmkit_set_warnings(swarmkit_vecr_trial)
swarmkit_enable_sanitizers(swarmkit_vecr_trial)

# The explicit object supplies all VECR symbols before static archive resolution.
# The normal core object is not pulled into this executable. No SDK/wire selector exists.
add_executable(swarmkit_vecr_baseline vecr_digital_trial.cpp ../src/core/vecr.cpp)
target_compile_definitions(swarmkit_vecr_baseline PRIVATE SWARMKIT_RESEARCH_FULL_RESTART=1)
target_link_libraries(swarmkit_vecr_baseline PRIVATE swarmkit_agent swarmkit_evidence swarmkit_client)
swarmkit_set_warnings(swarmkit_vecr_baseline)
swarmkit_enable_sanitizers(swarmkit_vecr_baseline)

add_executable(swarmkit_vecr_baseline_contract test_vecr_baseline.cpp ../src/core/vecr.cpp)
target_compile_definitions(swarmkit_vecr_baseline_contract PRIVATE SWARMKIT_RESEARCH_FULL_RESTART=1)
target_link_libraries(swarmkit_vecr_baseline_contract PRIVATE swarmkit_core Catch2::Catch2WithMain)
swarmkit_set_warnings(swarmkit_vecr_baseline_contract)
swarmkit_enable_sanitizers(swarmkit_vecr_baseline_contract)
add_test(NAME swarmkit_vecr_baseline_contract COMMAND swarmkit_vecr_baseline_contract)
