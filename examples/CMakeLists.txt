# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

add_executable(swarmkit-evidence-consumer-example
  ${CMAKE_CURRENT_SOURCE_DIR}/evidence_consumer.cpp
)

target_link_libraries(swarmkit-evidence-consumer-example PRIVATE swarmkit_client)
swarmkit_set_warnings(swarmkit-evidence-consumer-example)
swarmkit_enable_sanitizers(swarmkit-evidence-consumer-example)
