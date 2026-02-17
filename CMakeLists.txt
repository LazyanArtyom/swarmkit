cmake_minimum_required(VERSION 3.28)
project(swarmkit VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(SwarmkitOptions)
include(SwarmkitWarnings)
include(SwarmkitSanitizers)

find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(Catch2 CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(ZLIB REQUIRED)

file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/generated/include/swarmkit/core
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/swarmkit/core/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/include/swarmkit/core/version.h
)

add_subdirectory(proto)
add_subdirectory(src/core)
add_subdirectory(src/agent)
add_subdirectory(apps)

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()