# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

cmake_minimum_required(VERSION 3.24)
project(SwarmKitSdkProbe CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_property(is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT is_multi_config AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(SwarmKit REQUIRED)

add_executable(swarmkit-sdk-ping sdk_ping_probe.cpp)
target_link_libraries(swarmkit-sdk-ping PRIVATE swarmkit::client)
