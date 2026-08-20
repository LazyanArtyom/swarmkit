# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

add_executable(swarmkit-agent
    ${CMAKE_CURRENT_SOURCE_DIR}/agent/app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/agent/configuration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/agent/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/agent/usage.cpp
)
target_include_directories(swarmkit-agent PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(swarmkit-agent PRIVATE swarmkit_agent)
swarmkit_set_warnings(swarmkit-agent)
swarmkit_enable_sanitizers(swarmkit-agent)

add_executable(swarmkit-cli
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/command_builder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/data_artifact_runtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/data_message_runtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/flight_command_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/nav_command_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/options.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/output.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/payload_command_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/runtime_authority.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/runtime_command.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/runtime_goal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/runtime_health.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/runtime_reports.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/runtime_swarm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/runtime_telemetry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/runtime.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/telemetry_sink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cli/usage.cpp
)
target_include_directories(swarmkit-cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(swarmkit-cli PRIVATE swarmkit_client)
swarmkit_set_warnings(swarmkit-cli)
swarmkit_enable_sanitizers(swarmkit-cli)

add_executable(swarmkit-evidence-inspect
    ${CMAKE_CURRENT_SOURCE_DIR}/evidence_inspect.cpp
)
target_link_libraries(swarmkit-evidence-inspect PRIVATE swarmkit_evidence)
swarmkit_set_warnings(swarmkit-evidence-inspect)
swarmkit_enable_sanitizers(swarmkit-evidence-inspect)

add_executable(swarmkit-dissertation-experiment
    ${CMAKE_CURRENT_SOURCE_DIR}/experiment/dissertation_experiment.cpp
)
execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE SWARMKIT_EXPERIMENT_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE SWARMKIT_EXPERIMENT_GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(SWARMKIT_EXPERIMENT_GIT_STATUS STREQUAL "")
    set(SWARMKIT_EXPERIMENT_GIT_DIRTY false)
else()
    set(SWARMKIT_EXPERIMENT_GIT_DIRTY true)
endif()
target_compile_definitions(swarmkit-dissertation-experiment PRIVATE
    SWARMKIT_EXPERIMENT_GIT_COMMIT="${SWARMKIT_EXPERIMENT_GIT_COMMIT}"
    SWARMKIT_EXPERIMENT_GIT_DIRTY=${SWARMKIT_EXPERIMENT_GIT_DIRTY}
    SWARMKIT_EXPERIMENT_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
    SWARMKIT_EXPERIMENT_COMPILER_ID="${CMAKE_CXX_COMPILER_ID}"
    SWARMKIT_EXPERIMENT_COMPILER_VERSION="${CMAKE_CXX_COMPILER_VERSION}"
)
target_link_libraries(swarmkit-dissertation-experiment PRIVATE swarmkit_experiment)
swarmkit_set_warnings(swarmkit-dissertation-experiment)
swarmkit_enable_sanitizers(swarmkit-dissertation-experiment)

if(BUILD_TESTING)
    add_test(NAME swarmkit_evidence_inspect_help
        COMMAND swarmkit-evidence-inspect --help
    )
    set_tests_properties(swarmkit_evidence_inspect_help PROPERTIES
        PASS_REGULAR_EXPRESSION "usage: .* EXECUTION_LOG"
    )
endif()
