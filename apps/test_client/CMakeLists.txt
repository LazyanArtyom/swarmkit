# Standalone consumer project — links against an *installed* SwarmKit SDK.
# Does NOT use Conan. The SDK root is passed via CMAKE_PREFIX_PATH.
#
# Usage (after extracting the SDK tarball to e.g. ~/swarmkit-sdk):
#
#   cmake -B build \
#         -DCMAKE_PREFIX_PATH=~/swarmkit-sdk \
#         -DCMAKE_BUILD_TYPE=Release
#   cmake --build build
#   ./build/swarmkit-test-ping [agent_address]   # default: 127.0.0.1:50061

cmake_minimum_required(VERSION 3.24)
project(SwarmKitTestClient CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Locate the installed SwarmKit package.
# SwarmKitConfig.cmake (inside the SDK) discovers and loads all bundled deps.
find_package(SwarmKit REQUIRED)

add_executable(swarmkit-test-ping main.cpp)
target_link_libraries(swarmkit-test-ping PRIVATE swarmkit::client)
