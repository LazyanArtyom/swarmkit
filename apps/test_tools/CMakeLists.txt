# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

cmake_minimum_required(VERSION 3.24)
project(SwarmKitTestTools CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SwarmKit REQUIRED)
find_package(Threads REQUIRED)

# --- swarmkit-test-agents ----------------------------------------------------
add_executable(swarmkit-test-agents test_agents.cpp)
target_link_libraries(swarmkit-test-agents PRIVATE swarmkit::core Threads::Threads)

# --- swarmkit-test-client ----------------------------------------------------
add_executable(swarmkit-test-client test_client.cpp)
target_link_libraries(swarmkit-test-client PRIVATE swarmkit::client Threads::Threads)

# --- swarmkit-test-server ----------------------------------------------------
add_executable(swarmkit-test-server test_server.cpp)
target_link_libraries(swarmkit-test-server PRIVATE swarmkit::client Threads::Threads)
