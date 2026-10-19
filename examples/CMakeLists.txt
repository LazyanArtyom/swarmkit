# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

add_executable(swarmkit-edge-consumer-example
  ${CMAKE_CURRENT_SOURCE_DIR}/edge_consumer.cpp
)

target_link_libraries(swarmkit-edge-consumer-example PRIVATE swarmkit_client)
swarmkit_set_warnings(swarmkit-edge-consumer-example)
swarmkit_enable_sanitizers(swarmkit-edge-consumer-example)
