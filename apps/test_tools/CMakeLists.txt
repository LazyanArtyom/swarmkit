# Standalone integration / demo tools — link against an *installed* SwarmKit SDK.
# Does NOT use Conan. The SDK root is passed via CMAKE_PREFIX_PATH.
#
# Usage (after extracting the SDK tarball):
#
#   cmake -B build \
#         -DCMAKE_PREFIX_PATH=~/swarmkit-sdk \
#         -DCMAKE_BUILD_TYPE=Release \
#         -S apps/test_tools
#   cmake --build build
#
# Produced binaries:
#   swarmkit-test-agents  — spawns 3 agent processes, muxes their output.
#                           Requires the swarmkit-agent binary on PATH or
#                           passed as the first argument.
#                           (macOS / Linux only — uses POSIX fork/exec/pipe)
#
#   swarmkit-test-client  — connects 3 low-priority clients (kOperator=0) to
#                           the 3 agents and sends a rotating command sequence.
#                           Demonstrates preemption: CLI (kSupervisor=10) and
#                           test-server (kOverride=20) will override it.
#
#   swarmkit-test-server  — connects a single SwarmClient at kOverride (20)
#                           priority to all 3 agents, subscribes to telemetry,
#                           writes frames to a CSV file, and accepts interactive
#                           commands from stdin.

cmake_minimum_required(VERSION 3.24)
project(SwarmKitTestTools CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SwarmKit REQUIRED)
find_package(Threads REQUIRED)

# --- swarmkit-test-agents (POSIX only) ---------------------------------------
if (NOT WIN32)
    add_executable(swarmkit-test-agents test_agents.cpp)
    target_link_libraries(swarmkit-test-agents PRIVATE swarmkit::core Threads::Threads)
endif()

# --- swarmkit-test-client ----------------------------------------------------
add_executable(swarmkit-test-client test_client.cpp)
target_link_libraries(swarmkit-test-client PRIVATE swarmkit::client Threads::Threads)

# --- swarmkit-test-server ----------------------------------------------------
add_executable(swarmkit-test-server test_server.cpp)
target_link_libraries(swarmkit-test-server PRIVATE swarmkit::client Threads::Threads)
